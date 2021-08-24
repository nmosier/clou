#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>
#include <fstream>

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
   inst(inst), po(c.make_bool()),
   tfo(c.make_bool()), tfo_depth(c.make_int()),
   xsread(c.FALSE), xswrite(c.FALSE),
   constraints() {}


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
      ss << node.inst << "\n";
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

   simplify();

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
   std::unordered_map<std::string, unsigned> names;
   graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
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
   case z3::sat: {
      llvm::errs() << "sat";
      const z3::model model = solver.get_model();
      output_execution("out/exec.dot", model);
      break;
   }
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
   logv(2) << "Constructing comx\n";
   construct_comx();
}

void AEG::construct_nodes_addr_defs() {
   for (Node& node : nodes) {
      if (node.inst.addr_def) {
         node.addr_def = UHBAddress {context};
      }
   }
}

void AEG::construct_nodes_addr_refs() {
   std::unordered_map<const llvm::Argument *, UHBAddress> main_args;
   
   for (NodeRef ref = 0; ref < size(); ++ref) {
      const AEGPO2::Node& po_node = po.lookup(ref);
      Node& node = lookup(ref);
      for (const llvm::Value *V : node.inst.addr_refs) {
         const auto defs_it = po_node.refs.find(V);
         std::optional<UHBAddress> e;
         // z3::expr e {context.context};
         if (defs_it == po_node.refs.end()) {
            const llvm::Argument *A = llvm::cast<llvm::Argument>(V);
            auto main_args_it = main_args.find(A);
            if (main_args_it == main_args.end()) {
               main_args_it = main_args.emplace(A, UHBAddress {context}).first;
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
               e = UHBAddress {context};
               if (defs.size() != 0) {
                  node.constraints(util::any_of<z3::expr>(defs.begin(), defs.end(),
                                                          [&] (NodeRef def) {
                                                             return lookup_def(def).po == e->po;
                                                          }, context.FALSE));
                  node.constraints(util::any_of<z3::expr>(defs.begin(), defs.end(),
                                                          [&] (NodeRef def) {
                                                             return lookup_def(def).tfo == e->tfo;
                                                          }, context.FALSE));
               }
            }
         }
         node.addr_refs.emplace_back(V, *e);
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
         addrs.push_back({id, V, node.addr_def->po});
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
               addrs.push_back({id, V, it->second.po});
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

#if 0
std::optional<z3::expr> AEG::check_no_intervening_writes(NodeRef src, NodeRef dst) const {
   /* TODO: 
    * Visit the nodes in postorder, so that we know the full constraints of each node before
    * visiting successors, in order to avoid path explosion.
    *
    */
   
   z3::expr acc = context.TRUE;
   const Node& node = lookup(src);
   if (node.inst.kind == Inst::WRITE) {
      acc = z3::implies(node.po, node.addr_refs.at(0).po != lookup(src).addr_refs.at(0).po)
   }
   for (NodeRef pred : po.po.rev.at(src)) {
      if (pred != dst) {
         acc &= check_no_intervening_writes(pred, dst);
      }
   }
   return acc;
}
#else
z3::expr AEG::check_no_intervening_writes(NodeRef read, NodeRef write) const {
   /* Approach:
    * Use the postorder to be able to efficiently generate subpredicates.
    * For nodes of which `read` is an ancestor, it should all be false.
    * For nodes that are an ancestor of `read`, it should be computed accordingly.
    * All nodes start out as nullopt. All children must be nullopt to propogate.
    */
   const Node& read_node = lookup(read);
   std::vector<NodeRef> order;
   po.postorder(std::back_inserter(order));
   std::unordered_map<NodeRef, std::optional<z3::expr>> fs;
   for (NodeRef ref : order) {
      const auto& succs = po.po.fwd.at(ref);
      std::optional<z3::expr> out;
      for (NodeRef succ : succs) {
         if (const auto& in = fs.at(succ)) {
            out = out ? (*in && *out) : *in;
         }
      }
      if (out) {
         const Node& ref_node = lookup(ref);
         if (ref_node.inst.kind == Inst::WRITE) {
            *out &= lookup(ref).addr_refs.at(0).second.po != read_node.addr_refs.at(0).second.po;
         }
      } else {
         if (ref == read) {
            out = context.TRUE;
         }
      }
      fs.emplace(ref, out);
   }
   const auto& write_succs = po.po.fwd.at(write);
   return util::all_of(write_succs.begin(), write_succs.end(), [&] (NodeRef succ) -> z3::expr {
      const auto x = fs.at(succ);
      return x ? *x : context.TRUE;
   }, context.TRUE);
}
#endif
                                          
void AEG::construct_com() {
   assert(po.nodes.size() == nodes.size());

   // get reads and writes
   std::vector<NodeRef> reads;
   std::vector<NodeRef> writes;
   for (NodeRef ref : node_range()) {
      switch (lookup(ref).inst.kind) {
      case Inst::READ:
         reads.push_back(ref);
         break;
      case Inst::WRITE:
         writes.push_back(ref);
         break;
      default: break;
      }
   }
   
   /* construct rf */
   for (NodeRef ref : reads) {
      Node& node = lookup(ref);
      if (node.inst.kind == Inst::READ) {
         /* get set of possible writes */
         std::vector<CondNode> writes;
         find_sourced_memops(Inst::WRITE, ref, std::back_inserter(writes));

         /* add edges */
         for (const CondNode& write : writes) {
            Edge e {Edge::RF, context};
            e.exists = node.po && write.cond;
            graph.insert(write.ref, ref, e);
         }
      }

      std::vector<std::shared_ptr<Edge>> rfs;
      get_incoming_edges(ref, std::back_inserter(rfs), Edge::RF);
      const z3::expr one = util::one_of(rfs.begin(), rfs.end(), [] (const auto& ep) {
         return ep->exists;
      }, context.TRUE, context.FALSE);
      node.constraints(one);
   }
   
   /* construct co */
   for (NodeRef ref = 0; ref < po.nodes.size(); ++ref) {
      const Node& node = lookup(ref);
      if (node.inst.kind == Inst::WRITE) {
         /* get set of possible writes */
         std::vector<CondNode> writes;
         find_preceding_memops(Inst::WRITE, ref, std::back_inserter(writes));

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
         find_preceding_memops(Inst::READ, ref, std::back_inserter(reads));

         /* add edges */
         for (const CondNode& read : reads) {
            Edge e {Edge::FR, context};
            e.exists = node.po && read.cond;
            graph.insert(read.ref, ref, e);
         }
      }
   }
}


#if 0
template <typename OutputIt>
void AEG::find_sourced_memops(Inst::Kind kind, NodeRef org, OutputIt out) const {
   const Node& org_node = lookup(org);
   const z3::expr& org_addr = org_node.addr_refs.at(0).second.po;
   
   std::deque<CondNode> todo;
   const auto& init_preds = po.po.rev.at(org);
   std::transform(init_preds.begin(), init_preds.end(), std::front_inserter(todo),
                  [&] (NodeRef ref) {
                     return CondNode {ref, context.TRUE};
                  });

   while (!todo.empty()) {
      const CondNode& cn = todo.back();
      const Node& node = lookup(cn.ref);
      const auto& preds = po.po.rev.at(cn.ref);
      if (node.inst.kind == kind) {
         const z3::expr same_addr = org_addr == node.addr_refs.at(0).second.po;
         const z3::expr path_taken = node.po;
         *out++ = CondNode {cn.ref, cn.cond && same_addr && path_taken};
         for (NodeRef pred : preds) {
            todo.push_front({pred, cn.cond && !same_addr && path_taken});
         }
      } else {
         std::transform(preds.begin(), preds.end(), std::front_inserter(todo),
                        [&] (NodeRef pred) -> CondNode {
                           return {pred, cn.cond};
                        });
      }

      
      todo.pop_back();
   }
                  
}
#else
template <typename OutputIt>
void AEG::find_sourced_memops(Inst::Kind kind, NodeRef org, OutputIt out) const {
   std::unordered_map<NodeRef, std::optional<z3::expr>> nos;
   std::unordered_map<NodeRef, std::optional<z3::expr>> yesses;
   std::vector<NodeRef> order;
   po.postorder(std::back_inserter(order));

   const Node& org_node = lookup(org);
   for (NodeRef ref : order) {
      std::optional<z3::expr> out;
      for (NodeRef succ : po.po.fwd.at(ref)) {
         if (const auto& in = nos.at(succ)) {
            out = out ? (*out && *in) : *in;
         }
      }
      std::optional<z3::expr> yes;
      std::optional<z3::expr> no;
      if (out) {
         const Node& ref_node = lookup(ref);
         if (ref_node.inst.kind == kind) {
            const z3::expr same_addr =
               ref_node.addr_refs.at(0).second.po == org_node.addr_refs.at(0).second.po;
            yes = *out && ref_node.po && same_addr;
            no = *out && z3::implies(ref_node.po, !same_addr);
         } else {
            yes = std::nullopt;
            no = out;
         }
      } else if (ref == org) {
         yes = std::nullopt;
         no = context.TRUE;
      } else {
         yes = std::nullopt;
         no = std::nullopt;
      }
      nos.emplace(ref, no);
      yesses.emplace(ref, yes);
   }
   for (const auto& yes : yesses) {
      if (yes.second) {
         *out++ = {yes.first, *yes.second};
      }
   }
}
#endif

template <typename OutputIt>
void AEG::find_preceding_memops(Inst::Kind kind, NodeRef write, OutputIt out) const {
   const Node& write_node = lookup(write);
   const z3::expr& write_addr = write_node.addr_refs.at(0).second.po;

   std::deque<NodeRef> todo;
   std::unordered_set<NodeRef> seen;
   const auto& init_preds = po.po.rev.at(write);
   std::copy(init_preds.begin(), init_preds.end(), std::front_inserter(todo));

   while (!todo.empty()) {
      NodeRef ref = todo.back();
      todo.pop_back();
      
      if (!seen.insert(ref).second) {
         continue;
      }
      
      const Node& node = lookup(ref);
      if (node.inst.kind == kind) {
         const z3::expr same_addr = write_addr == node.addr_refs.at(0).second.po;
         const z3::expr path_taken = node.po;
         *out++ = CondNode {ref, same_addr && path_taken};
      }
      const auto& preds = po.po.rev.at(ref);
      std::copy(preds.begin(), preds.end(), std::front_inserter(todo));
   }
}


void AEG::construct_comx() {
   /* Set xsread, xswrite */
   std::unordered_set<NodeRef> xswrites;
   std::unordered_set<NodeRef> xsreads;
   for (NodeRef i = 0; i < size(); ++i) {
      Node& node = lookup(i);
      switch (node.inst.kind) {
      case Inst::READ:
         node.xsread = context.TRUE;
         node.xswrite = context.make_bool();
         xsreads.insert(i);
         xswrites.insert(i);
         // TODO: need to constrain when this happens
         break;
      case Inst::WRITE:
         node.xsread = context.TRUE;
         node.xswrite = context.TRUE;
         xsreads.insert(i);
         xswrites.insert(i);
         break;
      default:
         break;
      }
   }


   /* add rfx */
   /* We could just blindly add all possible combinations and then later build on this to reduce
    * the number of possible cycles.
    */

   /* Or, alternatively, do it in a more intelligent way: create all possible edges, attach
    * a boolean to them, and then enforce that there is exactly one incoming node with this 
    * edge.
    *
    * Basically, you construct the skeleton manually, and then use FOL/Alloy-like operators to
    * put constraints on these.
    * 
    * For example, rfx. We construct an overapproximation of the rfx relation -- add edges from 
    * a read to all writes of the same xstate.
    * Then, apply the FOL invariant that XSReads have exactly one incoming rfx edge.
    * fol::FORALL XSRead r | fol::ONE XSWrite w (w - rfx -> r)
    */
   
   for (NodeRef xsread : xsreads) {
      const Node& xsr = lookup(xsread);
      for (NodeRef xswrite : xswrites) {
         const Node& xsw = lookup(xswrite);
         const z3::expr path = xsr.get_exec() && xsw.get_exec();
         const z3::expr is_xstate = xsr.xsread && xsw.xswrite;
         const z3::expr same_xstate = xsr.same_xstate(xsw);
         const z3::expr exists = path && is_xstate && same_xstate;
         add_optional_edge(xswrite, xsread, UHBEdge {UHBEdge::RFX, exists});
      }
      
      // add special rfx edges with ENTRY
      add_optional_edge(entry, xsread, UHBEdge {UHBEdge::RFX, xsr.get_exec() && xsr.xsread});
   }

   /* rfx constraint: 
    * for any edge from po to tfo, require that the po node must be an ancestor of the tfo node.
    */
#if 1
   graph.for_each_edge([&] (NodeRef write, NodeRef read, Edge& e) {
      if (e.kind == Edge::RFX) {
         const Node& nw = lookup(write);
         const Node& nr = lookup(read); 
         const bool is_anc = is_ancestor(write, read);
         e.constraints(z3::implies(e.exists,
                                   z3::implies(nw.po && nr.tfo, context.to_expr(is_anc))));
      }
   });
#endif
   

   /* Constrain rfx edges */
#if 1
   for (NodeRef xsread : xsreads) {
      std::vector<std::shared_ptr<Edge>> es;
      get_incoming_edges(xsread, std::back_inserter(es), UHBEdge::RFX);
      const z3::expr constr = util::one_of(es.begin(), es.end(), [] (const auto & e) {
         return e->exists;
      }, context.TRUE, context.FALSE);
      lookup(xsread).constraints(constr);
   }
#endif

   /* Adding cox is easy -- just predicate edges on po/tfo of each node and whether they access
    * the same xstate.
    * How to select which nodes to consider? 
    * Considering all will cause an edge blowup. 
    */

   /* add cox */
   for (auto it1 = xswrites.begin(); it1 != xswrites.end(); ++it1) { 
      const Node& n1 = lookup(*it1);
      for (auto it2 = std::next(it1); it2 != xswrites.end(); ++it2) {
         const Node& n2 = lookup(*it2);
         const z3::expr path = n1.get_exec() && n2.get_exec();
         const z3::expr is_xstate = n1.xswrite && n2.xswrite;
         const z3::expr same_xstate = n1.same_xstate(n2);
         const z3::expr exists = path && is_xstate && same_xstate;
         add_bidir_edge(*it1, *it2, UHBEdge {UHBEdge::COX, exists});
      }
      add_unidir_edge(entry, *it1, UHBEdge {UHBEdge::COX, n1.get_exec() && n1.xswrite});
   }

   /* READs only perform an XSWrite if there are no previous READ/WRITEs to that address. 
    * When do we need competing pairs, anyway?
    * Only with upstream instructions.
    * For now, just use a hard-coded limit.
    *
    * Also, for now don't constrain the XSWrite boolean for READs. We can deal with this later.
    * We need to somehow enforce with comx that tfo executes before po.
    * 
    * For each node:
    * First consider po case: 
    */


   std::vector<graph_type::Cycle> cycles;
   graph.cycles(std::back_inserter(cycles), [] (const UHBEdge& e) -> bool {
      switch (e.kind) {
      case UHBEdge::PO:
      case UHBEdge::TFO:
      case UHBEdge::COX:
      case UHBEdge::RFX:
      case UHBEdge::FRX:
         return true;
      default:
         return false;
      }
   });
   for (const auto& cycle : cycles) {
      const auto f =
         std::transform_reduce(cycle.edges.begin(), cycle.edges.end(), context.FALSE,
                               util::logical_or<z3::expr>(),
                               [&] (const std::vector<UHBEdge>& es) -> z3::expr {
                                  return !std::transform_reduce(es.begin(), es.end(), context.FALSE,
                                                                util::logical_or<z3::expr>(),
                                                                [] (const UHBEdge& e) {
                                                                   return e.exists;
                                                                });

                               });
      constraints(f);
      logv(2) << util::to_string(f) << "\n";
   }
}

#if 0
template <typename OutputIt>
void AEG::find_comx_window(NodeRef ref, unsigned distance, unsigned spec_depth,
                           OutputIt out) const {
   /* Find predecessors exactly distance away */
   struct Entry {
      NodeRef ref;
      unsigned distance;
   };
   
   std::deque<Entry> po_todo {{ref, distance}};
   std::unordered_set<NodeRef> tfo_todo;
   std::unordered_set<NodeRef> po_set;
   std::unordered_set<NodeRef> tfo_set;

   // first explore po
   while (!todo.empty()) {
      const Entry& ent = todo.back();

      if (ent.distance == 0) {
         continue;
      }

      // output this node
      po_set.insert(ent.ref);

      const auto& preds = po.po.rev.at(ent.ref);
      std::transform(preds.begin(), preds.end(), std::front_inserter(todo),
                     [&] (NodeRef pred) -> Entry {
                        return {pred, distance - 1};
                     });

      todo.pop_back();
   }

   // now explore tfo
   
   for (NodeRef ref : po_set) {
      
   }

   std::copy(set.begin(), set.end(), out);
}
#endif

void AEG::add_bidir_edge(NodeRef a, NodeRef b, const UHBEdge& e) {
   UHBEdge e1 = e;
   UHBEdge e2 = e;
   const z3::expr dir = context.make_bool();
   e1.exists &=  dir;
   e2.exists &= !dir;
   add_unidir_edge(a, b, e1);
   add_unidir_edge(b, a, e2);
}

void AEG::add_optional_edge(NodeRef src, NodeRef dst, const UHBEdge& e_) {
   UHBEdge e = e_;
   const z3::expr constr = e.exists;
   e.exists = context.make_bool();
   e.constraints(z3::implies(e.exists, constr));
   graph.insert(src, dst, e);
}

std::ostream& operator<<(std::ostream& os, const UHBConstraints& c) {
   switch (c.exprs.size()) {
   case 0:
      break;
   case 1:
      os << c.exprs.front();
      break;
   default:
      os << std::reduce(std::next(c.exprs.begin()), c.exprs.end(), c.exprs.front(),
                        [] (const z3::expr& a, const z3::expr& b) -> z3::expr {
                           return a && b;
                        });
      break;
   }
   return os;
}


void AEG::output_execution(std::ostream& os, const z3::model& model) const {
   // using GNode = AEGPO2::Node::Variant;
   // using GEdge = UHBEdge::Kind;

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
      if (model.eval(node.get_exec()).bool_value() == Z3_L_TRUE) {
         const std::string name = std::string("n") + std::to_string(next_id);
         names.emplace(ref, name);
         ++next_id;

         os << name << " ";
         std::stringstream ss;
         ss << node.inst << "\n";

         switch (node.inst.kind) {
         case Inst::WRITE:
         case Inst::READ:
            ss << "{" << model.eval(node.get_addr_ref(0)) << "}"
               << "\n";
            break;
         default: break;
         }
         
         dot::emit_kvs(os, "label", ss.str());
         os << ";\n";
      }
   }

   graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
      if (model.eval(edge.exists).bool_value() == Z3_L_TRUE) {
         os << names.at(src) << " -> " << names.at(dst) << " ";
         dot::emit_kvs(os, "label", util::to_string(edge.kind));
         os << ";\n";
      }
   });

   os << "}\n";
}

void AEG::output_execution(const std::string& path, const z3::model& model) const {
   std::ofstream ofs {path};
   output_execution(ofs, model);
}

bool AEG::is_ancestor(NodeRef parent, NodeRef child) const {
   return is_ancestor_a(parent, child);
}

bool AEG::is_ancestor_a(NodeRef parent, NodeRef child) const {
   if (parent == child) {
      return true;
   }
   bool acc = false;
   for (NodeRef pred : po.po.rev.at(child)) {
      acc = acc || is_ancestor_a(parent, pred);
   }
   return acc;
}


bool AEG::is_ancestor_b(NodeRef parent, NodeRef child) const {
   std::vector<NodeRef> order;
   po.postorder(std::back_inserter(order));
   const auto child_it = std::find(order.begin(), order.end(), child);
   const auto parent_it = std::find(order.begin(), order.end(), parent);
   if (parent_it >= child_it) {
      return is_ancestor_a(parent, child);
   } else {
      // parent comes before child in postorder, so can't possibly be an ancestor
      return false;
   }
}

