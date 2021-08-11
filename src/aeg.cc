#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>

#include "z3-util.h"
#include "aeg.h"
#include "mcfg.h"
#include "aeg-po.h"

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
UHBNode::UHBNode(CFG::NodeRef ref, const Inst& inst, UHBContext& c):
   cfg_ref(ref), inst(inst), po(c.make_bool()), tfo(c.make_bool()), tfo_depth(c.make_int()),
   constraints(c) {}

void AEG::construct(const AEGPO& po, unsigned spec_depth, llvm::AliasAnalysis& AA) {
   // initialize nodes
   std::transform(po.nodes.begin(), po.nodes.end(), std::back_inserter(nodes),
                  [&] (const AEGPO::Node& node) {
                     const Inst inst =
                        std::visit(util::creator<Inst>(), po.cfg.lookup(node.cfg_ref).v);
                     return Node {node.cfg_ref, inst, context};
                  });
   for (NodeRef ref : node_range()) {
      graph.add_node(ref);
   }
   
   construct_nodes_po(po);
   construct_nodes_tfo(po, spec_depth);
   construct_edges_po_tfo(po);
   construct_aliases(po, AA);
}

void AEG::construct_nodes_po(const AEGPO& po) {
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

void AEG::construct_nodes_tfo(const AEGPO& po, unsigned spec_depth) {
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
         ss_ << cfg.lookup(node.cfg_ref);
         ss << s << "\n";
      }
      ss << "po: " << node.po << "\n"
         << "tfo: " << node.tfo << "\n"
         << "tfo_depth: " << node.tfo_depth << "\n";
      if (node.addr) {
         ss << "addr: " << *node.addr << "\n";
      }
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
   std::for_each(nodes.begin(), nodes.end(), [] (Node& node) { node.simplify(); });
   constraints.simplify();
   graph.for_each_edge([] (NodeRef, NodeRef, Edge& edge) {
      edge.simplify();
   });
}


void AEG::construct_edges_po_tfo(const AEGPO& po) {
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

void AEG::construct_aliases(const AEGPO& po, llvm::AliasAnalysis& AA) {
   /* assign address variables (symbolic ints) to each node */
   for (NodeRef ref : node_range()) {
      Node& node = lookup(ref);
      assert(!node.addr);
      if (node.inst.addr) {
         assert(node.inst.kind != Inst::Kind::EXIT);
         node.addr = context.make_int();
      }
   }

   /* generate constraints for each node */
   for (NodeRef ref1 : node_range()) {
      Node& node1 = lookup(ref1);
      if (node1.addr) {
         for (NodeRef ref2 : node_range()) {
            if (ref1 != ref2) {
               const Node& node2 = lookup(ref2);
               if (node2.addr) {
                  if (node1.inst.I == node2.inst.I) { // same instruction, special case
                  } else { // different instructions, default to builtin alias analysis results
                     switch (AA.alias(node1.inst.addr, node2.inst.addr)) {
                     case llvm::MustAlias:
                        node1.constraints(*node1.addr == *node2.addr);
                        break;
                     case llvm::MayAlias:
                     case llvm::PartialAlias:
                        break;
                     case llvm::NoAlias:
                        node1.constraints(*node1.addr != *node2.addr);
                     default: std::abort();
                     }
                  }
               }
            }
         }
      }
   }
   
   
#if 0
      // generate set of addresses
      std::unordered_map<const llvm::Value *, z3::expr> addrs;
      for (NodeRef ref : node_range()) {
         Node& node = lookup(ref);
         if (node.inst.addr) {
            assert(node.inst.kind != Inst::Kind::EXIT);
            auto it = addrs.find(node.inst.I);
            if (it == addrs.end()) {
               it = addrs.emplace(node.inst.addr, context.make_int()).first;
            }
            node.addr = it->second;
         }
      }

      // generate constraints
      for (auto it1 = addrs.begin(); it1 != addrs.end(); ++it1) {
         for (auto it2 = addrs.begin(); it2 != it1; ++it2) {
            switch (AA.alias(it1->first, it2->first)) {
            case llvm::MustAlias:
               constraints(it1->second == it2->second);
               break;
            case llvm::MayAlias:
            case llvm::PartialAlias:
               break;
            case llvm::NoAlias:
               constraints(it1->second != it2->second);
               break;
            }
         }
      }
#endif
}
