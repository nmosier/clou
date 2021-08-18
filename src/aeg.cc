#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>

#include "z3-util.h"
#include "aeg.h"
#include "mcfg.h"
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

#if 0
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


#if 0
AEG::NodeRef AEG::find_upstream_def(NodeRef node, const llvm::Value *addr_ref) const {
   /* Use BFS */
   std::deque<NodeRef> queue;
   const auto& init_preds = po.po.rev.at(node);
   std::copy(init_preds.begin(), init_preds.end(), std::front_inserter(queue));

   while (!queue.empty()) {
      const NodeRef nr = queue.back();
      queue.pop_back();
      const Node& node = lookup(nr);
      if (node.inst.addr_def == addr_ref) {
         return nr;
      }
      const auto& preds = po.po.rev.at(nr);
      std::copy(preds.begin(), preds.end(), std::front_inserter(queue));
   }

   throw std::logic_error("address definition not found");
}
#endif

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
                                   node->v);
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
   logv(2) << "Constructing aliases\n";
   construct_aliases(AA);
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


