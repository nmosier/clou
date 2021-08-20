#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>

#include "z3-util.h"
#include "aeg.h"
#include "config.h"

/* TODO
 * [ ] Don't use seen when generating tfo constraints
 * [ ] Use Graph<>'s node iterator, since it skips over deleted nodes? Or improve node range
 *     to skip deleted nodes.
 */


std::ostream& operator<<(std::ostream& os, const UHBEdge& e) {
   os << e.kind_tostr() << " " << e.exists << "\n"
      << e.constraints << "\n";
   return os;
}

const char *UHBEdge::kind_tostr(Kind kind) {
#define UHBEDGE_KIND_CASE(name) case name: return #name;
   switch (kind) {
      UHBEDGE_KIND_X(UHBEDGE_KIND_CASE)
   default: return nullptr;
   }
#undef UHBEDGE_KIND_CASE
}

UHBEdge::Kind UHBEdge::kind_fromstr(const std::string& s) {
#define UHBEDGE_KIND_PAIR(name) {#name, name}, 
   static const std::unordered_map<std::string, Kind> map {
      UHBEDGE_KIND_X(UHBEDGE_KIND_PAIR)
   };
   return map.at(s);
#undef UHBEDGE_KIND_PAIR
}

/* PO
 * Create a fresh bool. Predecessors imply exactly one successor's bool is true.
 *
 * TFO
 * Create a fresh bool. Predecessors imply any number of successor's bool is true.
 *
 * po => tfo.
 * 
 * TFO must be at most n hops away from a po.
 * OR all nodes exactly distance n away together (or TOP, in the edge case).
 */
UHBNode::UHBNode(const Inst& inst, UHBContext& c):
   inst(inst), po(c.make_bool()), tfo(c.make_bool()), tfo_depth(c.make_int()),
   constraints(c) {}


void AEG::dump_graph(const std::string& path) const {
   std::error_code ec;
   llvm::raw_fd_ostream os {path, ec};
   if (ec) {
      llvm::errs() << ec.message() << "\n";
      std::exit(1);
   }
   dump_graph(os);
}

void AEG::dump_graph(llvm::raw_ostream& os) const {
   os << R"=(
digraph G {
  overlap = scale;
  splines = true;

)=";

   // define nodes
   unsigned next_id = 0;
   std::unordered_map<NodeRef, std::string> names;
   for (NodeRef ref : node_range()) {
      const Node& node = lookup(ref);
      const std::string name = std::string("n") + std::to_string(next_id);
      names.emplace(ref, name);
      ++next_id;

      os << name << " ";

      std::stringstream ss;
      ss << node.inst.kind_tostr() << "\n";
      {
         std::string s;
         llvm::raw_string_ostream ss_ {s};
         ss_ << node.inst;
         ss << s << "\n";
      }
      ss << "po: " << node.po << "\n"
         << "tfo: " << node.tfo << "\n"
         << "tfo_depth: " << node.tfo_depth << "\n";

#if 1
      if (node.addr_def) {
         ss << "addr (def): " << *node.addr_def << "\n";
      }
      if (!node.addr_refs.empty()) {
         ss << "addr (refs):";
         for (const auto& ref : node.addr_refs) {
            ss << " " << ref.second;
         }
         ss << "\n";
      }
#endif

      ss << "constraints: " << node.constraints << "\n";
      
      dot::emit_kvs(os, "label", ss.str());
      os << ";\n";
   }

   // define edges
   graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
      os << names.at(src) << " -> " << names.at(dst) << " ";
      dot::emit_kvs(os, "label", util::to_string(edge));
      os << ";\n";
   });

   // graph labels (constraints)
   {
      os << "graph ";
      std::stringstream ss;
      ss << constraints;
      dot::emit_kvs(os, "label", ss.str());
      os << "\n";
   }

   os << "}\n";
}


void AEG::simplify() {
   unsigned count = 0;
   std::for_each(nodes.begin(), nodes.end(), [&] (Node& node) {
      llvm::errs() << ++count << "\n";
      node.simplify();
   });
   constraints.simplify();
   graph.for_each_edge([] (NodeRef, NodeRef, Edge& edge) {
      edge.simplify();
   });
}


void AEG::test() {
   z3::solver solver {context.context};

   /* display stats */
   if (verbose >= 3) {
      auto& os = llvm::errs();
      os << constraints.exprs.size() << " top level constraints\n";
      const unsigned node_clauses =
         std::transform_reduce(nodes.begin(), nodes.end(), 0, std::plus<unsigned>(),
                               [] (const Node& node) {
                                  return node.constraints.exprs.size();
                               });
      os << node_clauses << " node constraints\n";
      unsigned edge_clauses = 0;
      graph.for_each_edge([&] (NodeRef, NodeRef, const Edge& e) {
         edge_clauses += e.constraints.exprs.size();
      });
      os << edge_clauses << " edge constraints\n";
   }
   

   // add edge constraints 
   graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
      std::stringstream ss;
      ss << src << "->" << dst;
      edge.constraints.add_to(solver);
   });

   // add node constraints 
   for (NodeRef ref : node_range()) {
      lookup(ref).constraints.add_to(solver);
   }

   // add main constraints
   constraints.add_to(solver);

   // check node 15
   // solver.add(lookup(NodeRef {15}).po);
   
   switch (solver.check()) {
   case z3::unsat: {
      llvm::errs() << "unsat\n";
      const auto& core = solver.unsat_core();
      for (const auto& expr : core) {
         llvm::errs() << util::to_string(expr) << "\n";
      }
      break;
   }
   case z3::sat:     llvm::errs() << "sat"    ; break;
   case z3::unknown: llvm::errs() << "unknown"; break;
   }
   
}

/* Using alias analysis to construct address variables. 
 * - Must alias: S = T
 * - May alias:  (no constraint)
 * - No alias:   S != T
 * One important added rule to handle loops:
 *  - Self-alias checks always returns 'may alias' to generalize across loops.
 */

void AEG::find_upstream_def(NodeRef node, const llvm::Value *addr_ref,
                            std::unordered_set<NodeRef>& out) const {
   /* Use BFS */
   std::deque<NodeRef> queue;
   const auto& init_preds = po.po.rev.at(node);
   std::copy(init_preds.begin(), init_preds.end(), std::front_inserter(queue));

   while (!queue.empty()) {
      const NodeRef nr = queue.back();
      queue.pop_back();
      const Node& node = lookup(nr);
      if (node.inst.addr_def == addr_ref) {
         out.insert(nr);
      } else {
         const auto& preds = po.po.rev.at(nr);
         std::copy(preds.begin(), preds.end(), std::front_inserter(queue));
      }
   }
}

#if 0
template <typename Pred>
void AEG::for_each_pred(NodeRef ref, Pred pred) {
   std::deque<NodeRef> todo;
   const auto& init_preds = graph.rev.at(ref);
   std::copy(init_preds.begin(), init_preds.end(), std::front_inserter(todo));

   while (!todo.empty()) {
      const NodeRef ref = todo.back();
      todo.pop_back();
      if (pred(ref)) {
         const auto& preds = graph.rev.at(ref);
         std::copy(preds.begin(), preds.end(), std::front_inserter(todo));
      }
   }
}
#endif


void AEG::construct(unsigned spec_depth, llvm::AliasAnalysis& AA) {
   // initialize nodes
   std::transform(po.nodes.begin(), po.nodes.end(), std::back_inserter(nodes),
                  [&] (const auto& node) {
                     const Inst inst =
                        std::visit(util::creator<Inst>(),
                                   node.v);
                     return Node {inst, context};
                  });
   for (NodeRef ref : node_range()) {
      graph.add_node(ref);
   }

   logv(2) << "Constructing nodes po\n";
   construct_nodes_po();
   logv(2) << "Constructing nodes tfo\n";
   construct_nodes_tfo(spec_depth);
   logv(2) << "Constructing edges po tfo\n";
   construct_edges_po_tfo();
   logv(2) << "Constructing nodes addr defs\n";
   construct_nodes_addr_defs();
   logv(2) << "Constructing nodes addr refs\n";
   construct_nodes_addr_refs();
   logv(2) << "Constructing aliases\n";
   construct_aliases(AA);
   logv(2) << "Constructing com\n";
   construct_com();
}

void AEG::construct_nodes_addr_defs() {
   for (Node& node : nodes) {
      if (node.inst.addr_def) {
         node.addr_def = context.make_int();
      }
   }
}

void AEG::construct_nodes_addr_refs() {
   std::unordered_map<const llvm::Argument *, z3::expr> main_args;
   
   for (NodeRef ref = 0; ref < size(); ++ref) {
      const AEGPO2::Node& po_node = po.lookup(ref);
      Node& node = lookup(ref);
      for (const llvm::Value *V : node.inst.addr_refs) {
         const auto defs_it = po_node.refs.find(V);
         z3::expr e {context.context};
         if (defs_it == po_node.refs.end()) {
            const llvm::Argument *A = llvm::cast<llvm::Argument>(V);
            auto main_args_it = main_args.find(A);
            if (main_args_it == main_args.end()) {
               main_args_it = main_args.emplace(A, context.make_int()).first;
            }
            e = main_args_it->second;
         } else {
            const AEGPO2::NodeRefSet& defs = defs_it->second;

            /* If defs only has one element (likely case), then we can just lookup that element's 
             * address definition integer. Otherwise, we define a new symbolic int that must be equal 
             * to one of the possiblities.
             */
            const auto lookup_def = [&] (NodeRef def) {
               return *lookup(def).addr_def;
            };
            if (defs.size() == 1) {
               e = lookup_def(*defs.begin());
            } else {
               e = context.make_int();
               if (defs.size() != 0) {
                  node.constraints(util::any_of<z3::expr>(defs.begin(), defs.end(),
                                                          [&] (NodeRef def) {
                                                             return lookup_def(def) == e;
                                                          }, context.FALSE));
               }
            }
         }
         node.addr_refs.emplace_back(V, e);
      }
   }
}

void AEG::construct_nodes_po() {
   for (NodeRef ref : node_range()) {
      const auto& preds = po.po.rev.at(ref);
      const auto& succs = po.po.fwd.at(ref);
      Node& node = lookup(ref);

      if (preds.empty()) {
         // program entry, require po
         node.constraints(node.po);
      }

      /* add po constraint: exactly one successor */
      if (!succs.empty()) {
         // Not Program Exit
         const z3::expr succ_po = util::one_of(succs.begin(), succs.end(), [&] (NodeRef dstref) {
            return lookup(dstref).po;
         }, context.TRUE, context.FALSE);
         node.constraints(z3::implies(node.po, succ_po));
      }
      // po excludes tfo
      node.constraints(z3::implies(node.po, !node.tfo));
   }
}

void AEG::construct_nodes_tfo(unsigned spec_depth) {
   std::unordered_set<NodeRef> seen;

   std::deque<NodeRef> todo {po.entry};
   while (!todo.empty()) {
      const NodeRef noderef = todo.front();
      todo.pop_front();
      if (!seen.insert(noderef).second) { continue; }
      const auto& preds = po.po.rev.at(noderef);
      const auto& succs = po.po.fwd.at(noderef);
      std::copy(succs.begin(), succs.end(), std::back_inserter(todo));
      
      /* set tfo_depth */
      Node& node = lookup(noderef);
      if (preds.size() != 1) {
         node.constraints(node.tfo_depth == context.context.int_val(0));
         node.constraints(!node.tfo); // force TFO to false
         // NOTE: This covers both TOP and non-speculative join cases.
      } else {
         const Node& pred = lookup(*preds.begin());
         const z3::expr tfo_depth_expr =
            z3::ite(node.po,
                    context.context.int_val(0),
                    pred.tfo_depth + context.context.int_val(1));
         node.constraints(node.tfo_depth == tfo_depth_expr);
         node.constraints(z3::implies(node.tfo_depth > context.context.int_val(spec_depth),
                                      !node.tfo));
         node.constraints(z3::implies(node.tfo, pred.tfo || pred.po));
      }
   }
}

void AEG::construct_edges_po_tfo() {
   /* When does a po edge exist between two nodes?
    * (1) po must hold for both nodes.
    * (2) One must directly follow the other.
    *
    */

   for (NodeRef ref : node_range()) {
      const Node& node = lookup(ref);
      const auto& succs = po.po.fwd.at(ref);
      for (NodeRef succ_ref : succs) {
         const Node& succ = lookup(succ_ref);
         {
            UHBEdge edge {UHBEdge::PO, context};
            edge.constraints(edge.exists == (node.po && succ.po));
            graph.insert(ref, succ_ref, edge);
         }
         {
            UHBEdge edge {UHBEdge::TFO, context};
            edge.constraints(edge.exists == ((node.po || node.tfo) && succ.tfo));
            graph.insert(ref, succ_ref, edge);
         }
      }
   }
}

void AEG::construct_aliases(llvm::AliasAnalysis& AA) {
   using ID = AEGPO2::ID;
   struct Info {
      ID id;
      const llvm::Value *V;
      z3::expr e;
   };
   std::vector<Info> addrs;
   std::unordered_set<std::pair<ID, const llvm::Value *>> seen;
   for (NodeRef i = 0; i < size(); ++i) {
      const Node& node = lookup(i);
      if (node.addr_def) {
         const ID& id = *po.lookup(i).id;
         const llvm::Value *V = node.inst.I;
         addrs.push_back({id, V, *node.addr_def});
         [[maybe_unused]] const auto res = seen.emplace(id, V);
         assert(res.second);
      }
   }

   // check for arguments
   for (NodeRef i = 0; i < size(); ++i) {
      const Node& node = lookup(i);
      const AEGPO2::Node& po_node = po.lookup(i);
      for (const llvm::Value *V : node.inst.addr_refs) {
         if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
            const ID id {po_node.id->func, {}};
            if (seen.emplace(id, V).second) {
               const auto it = std::find_if(node.addr_refs.begin(), node.addr_refs.end(),
                                            [&] (const auto& p) {
                                               return p.first == V;
                                            });
               assert(it != node.addr_refs.end());
               addrs.push_back({id, V, it->second});
            }
         }
      }
   }
   
   // add constraints
   for (auto it1 = addrs.begin(); it1 != addrs.end(); ++it1) {
      for (auto it2 = std::next(it1); it2 != addrs.end(); ++it2) {
         if (po.alias_valid(it1->id, it2->id)) {
            const auto alias_res = AA.alias(it1->V, it2->V);
            switch (alias_res) {
            case llvm::NoAlias:
               constraints(it1->e != it2->e);
               break;
            case llvm::MayAlias:
               break;
            case llvm::MustAlias:
               constraints(it1->e == it2->e);
               break;
            default: std::abort();
            }
         }
      }
   }
}

void AEG::construct_com() {
   assert(po.nodes.size() == nodes.size());
   
   /* construct rf */
   for (NodeRef ref = 0; ref < po.nodes.size(); ++ref) {
      const Node& node = lookup(ref);
      if (node.inst.kind == Inst::READ) {
         /* get set of possible writes */
         std::vector<CondNode> writes;
         find_sourced_writes(ref, std::back_inserter(writes));

         /* add edges */
         for (const CondNode& write : writes) {
            Edge e {Edge::RF, context};
            e.exists = node.po && write.cond;
            graph.insert(write.ref, ref, e);
         }
      }
   }

   /* construct co */
   for (NodeRef ref = 0; ref < po.nodes.size(); ++ref) {
      const Node& node = lookup(ref);
      if (node.inst.kind == Inst::WRITE) {
         /* get set of possible writes */
         std::vector<CondNode> writes;
         find_preceding_writes(ref, std::back_inserter(writes));

         /* add edges */
         for (const CondNode& write : writes) {
            Edge e {Edge::CO, context};
            e.exists = node.po && write.cond;
            graph.insert(write.ref, ref, e);
         }
      }
   }
   
   /* construct fr 
    * This is computed as ~rf.co, I think.
    * But the easier way for now is to construct it directly, like above for rf and co.
    */
   for (NodeRef ref = 0; ref < po.nodes.size(); ++ref) {
      const Node& node = lookup(ref);
      if (node.inst.kind == Inst::WRITE) {
         /* get set of possible reads */
         std::vector<CondNode> reads;
         find_sourced_reads(ref, std::back_inserter(reads));

         /* add edges */
         for (const CondNode& read : reads) {
            Edge e {Edge::FR, context};
            e.exists = node.po && read.cond;
            graph.insert(read.ref, ref, e);
         }
      }
   }
}


template <Inst::Kind KIND, typename OutputIt>
void AEG::find_sourced_memops(NodeRef org, OutputIt out) const {
   const Node& org_node = lookup(org);
   const z3::expr& org_addr = org_node.addr_refs.at(0).second;
   
   std::deque<CondNode> todo;
   const auto& init_preds = po.po.rev.at(org);
   std::transform(init_preds.begin(), init_preds.end(), std::front_inserter(todo),
                  [&] (NodeRef ref) {
                     return CondNode {ref, context.TRUE};
                  });

   while (!todo.empty()) {
      const CondNode& cn = todo.back();
      const Node& node = lookup(cn.ref);

      if (node.inst.kind == KIND) {
         const z3::expr same_addr = org_addr == node.addr_refs.at(0).second;
         const z3::expr path_taken = node.po;
         *out++ = CondNode {cn.ref, cn.cond && same_addr && path_taken};
         for (NodeRef pred : po.po.rev.at(cn.ref)) {
            todo.push_back({pred, cn.cond && !same_addr && path_taken});
         }
      }
      
      todo.pop_back();
   }
                  
}



template <typename OutputIt>
void AEG::find_sourced_writes(NodeRef read, OutputIt out) const {
   find_sourced_memops<Inst::WRITE>(read, out);
}

template <typename OutputIt>
void AEG::find_sourced_reads(NodeRef write, OutputIt out) const {
   find_sourced_memops<Inst::READ>(write, out);
}

template <typename OutputIt>
void AEG::find_preceding_writes(NodeRef write, OutputIt out) const {
   const Node& write_node = lookup(write);
   const z3::expr& write_addr = write_node.addr_refs.at(0).second;

   std::deque<NodeRef> todo;
   const auto& init_preds = po.po.rev.at(write);
   std::copy(init_preds.begin(), init_preds.end(), std::front_inserter(todo));

   while (!todo.empty()) {
      NodeRef ref = todo.back();
      todo.pop_back();
      const Node& node = lookup(ref);
      if (node.inst.kind == Inst::WRITE) {
         const z3::expr same_addr = write_addr == node.addr_refs.at(0).second;
         const z3::expr path_taken = node.po;
         *out++ = CondNode {ref, same_addr && path_taken};
         const auto& preds = po.po.rev.at(ref);
         std::copy(preds.begin(), preds.end(), std::front_inserter(todo));
      }
   }
}


