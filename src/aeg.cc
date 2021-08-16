#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>

#include "z3-util.h"
#include "aeg.h"
#include "mcfg.h"
#include "aeg-po.h"
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

void AEG::construct(unsigned spec_depth, llvm::AliasAnalysis& AA) {
   // initialize nodes
   std::transform(po.nodes.begin(), po.nodes.end(), std::back_inserter(nodes),
                  [&] (const AEGPO2::Node& node) {
                     const Inst inst =
                        std::visit(util::creator<Inst>(),
                                   node());
                     return Node {inst, context};
                  });
   for (NodeRef ref : node_range()) {
      graph.add_node(ref);
   }

   if (verbose >= 2) { llvm::errs() << "Constructing nodes po\n"; }
   construct_nodes_po();
   if (verbose >= 2) { llvm::errs() << "Constructing nodes tfo\n"; }   
   construct_nodes_tfo(spec_depth);
   if (verbose >= 2) { llvm::errs() << "Constructing edges po tfo\n"; }
   construct_edges_po_tfo();
   if (verbose >= 2) { llvm::errs() << "Constructing aliases\n"; }
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

      /* tfo_depth[N], tfo[N]
       * 
       * 
       * 
       */

      /* set tfo_depth */
      Node& node = lookup(noderef);
      if (preds.size() != 1) {
         node.constraints(node.tfo_depth == context.context.int_val(0));
         node.constraints(!node.tfo); // force TFO to false
         // Why would we force TFO to false? Couldn't we still speculate?
         // It's because I was assuming we'd expand it beyond the speculation window size.
         // But we probably don't want to make that assumption.
         // Should actually force at most one misspeculated path. 
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

void AEG::construct_aliases(llvm::AliasAnalysis& AA) {
#if 0
   /* assign fresh symbolic variables to defs */
   std::unordered_map<const llvm::Value *, z3::expr> class_ids;
   for (Node& node : nodes) {
      const llvm::Value *def = node.inst.addr_def;
      if (def && class_ids.find(def) == class_ids.end()) {
         node.addr_def = context.make_int();
         class_ids.emplace(def, *node.addr_def);
      }
   }

   /* iterate over all class combinations */
   for (const auto& class1 : class_ids) {
      for (const auto& class2 : class_ids) {
         std::optional<std::function<z3::expr(const z3::expr&, const z3::expr&)>> f;
         switch (AA.alias(class1.first, class2.first)) {
         case llvm::MustAlias:
            constraints(class1.second == class2.second);
            f = [] (const z3::expr& a, const z3::expr& b) -> z3::expr { return a == b; };
            break;
         case llvm::NoAlias:
            f = [] (const z3::expr& a, const z3::expr& b) -> z3::expr { return a != b; };
            break;
         case llvm::MayAlias:
            break;
         default:
            std::abort();
         }

         if (f) {
            
         }
      }
   }

   if (verbose >= 3) {
      llvm::errs() << "Constructing aliases on " << nodes.size() << " nodes with ";
      unsigned n = 0;
      for (NodeRef ref : node_range()) {
         if (lookup(ref).addr_def) {
            ++n;
         }
      }
      llvm::errs() << n << " addresses\n";
   }

   /* construct address map */
   std::unordered_map<const llvm::Value *, std::unordered_set<NodeRef>> map;
   for (NodeRef ref : node_range()) {
      const Node& node = lookup(ref);
      if (node.inst.addr_def) {
         map[node.inst.addr_def].insert(ref);
      }
   }

   if (verbose >= 3) {
      llvm::errs() << map.size() << " distinct addresses\n";
   }

   /* iterate over all address combinations */
   for (const auto& P1 : map) {
      const llvm::Value *A1 = P1.first;
      const auto& refs1 = P1.second;
      for (const auto& P2 : map) {
         const llvm::Value *A2 = P2.first;
         const auto& refs2 = P2.second;
         std::optional<std::function<z3::expr(const z3::expr&, const z3::expr&)>> f;
         switch (AA.alias(A1, A2)) {
         case llvm::MustAlias:
            f = [] (const z3::expr& a, const z3::expr& b) -> z3::expr { return a == b; };
            break;
         case llvm::NoAlias:
            f = [] (const z3::expr& a, const z3::expr& b) -> z3::expr { return a != b; };
            break;
         case llvm::MayAlias:
            break;
         default:
            std::abort();
         }

         if (f) {
            for (NodeRef ref1 : refs1) {
               Node& node1 = lookup(ref1);
               for (NodeRef ref2 : refs2) {
                  Node& node2 = lookup(ref2);
                  node1.constraints((*f)(*node1.addr_def, *node2.addr_def));
               }
            }
         }
      }
   }

   // TODO: Assign addr_refs

   /*
    * 'No alias' and 'Must alias' only applies within a loop iteration, function call, 
    * For each instruction, find the deepest 
    *
    * Hmm. It incorrectly says 'no alias' across function calls.
    * So we can only assume results are correct within our current function call.
    * Results are valid up to function call and loop iteration.
    * 
    * Intra-loop same-iteration AA results are valid.
    * Intra-loop different-iteration AA results are possibly invalid.
    *
    * Need to assign nodes in AEG-PO to tightest containing loop or function.
    */
#endif
}


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
