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
