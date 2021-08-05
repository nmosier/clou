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
UHBNode::UHBNode(CFG::NodeRef ref, UHBContext& c):
   cfg_ref(ref), po(c.make_bool()), tfo(c.make_bool()), tfo_depth(c.make_int()),
   constraints(c) {}

void AEG::construct(const AEGPO& po, unsigned spec_depth, llvm::AliasAnalysis& AA) {
   // initialize nodes
   std::transform(po.nodes.begin(), po.nodes.end(), std::back_inserter(nodes),
                  [&] (const AEGPO::Node& node) {
                     return Node {node.cfg_ref, context};
                  });
   for (NodeRef ref : node_range()) {
      graph.add_node(ref);
   }
   
   construct_nodes_po(po);
   construct_nodes_tfo(po, spec_depth);
   construct_edges_po_tfo(po);
   construct_aliases(po.cfg, AA);
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
      {
         std::string s;
         llvm::raw_string_ostream ss_ {s};
         ss_ << cfg.lookup(node.cfg_ref);
         ss << s << "\n";
      }
      ss << "po: " << node.po << "\n"
         << "tfo: " << node.tfo << "\n"
         << "tfo_depth: " << node.tfo_depth << "\n"
         << "constraints: " << node.constraints << "\n"
         << "addrs: " << util::to_string(node.addrs.begin(), node.addrs.end()) << "\n";
      
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

/* How to construct address elements?
 * We can get alias information for each node. 
 * 1. Compute alias information of all previous. This is O(n^2) total.
 * - Must alias: points-to set equal to aliasee.
 * - May alias: points-to superset of aliasee.
 * - Partial alias: points-to equal to 
 */

void AEG::construct_aliases(const CFG& cfg, llvm::AliasAnalysis& AA) {
   using AddrMap = std::unordered_map<const llvm::Instruction *, UHBNode::AddrSet>;
   UHBAddress next_id = 0;
   AddrMap addrmap; // map CFG nodes to address sets

   for (CFG::NodeRef ref : cfg.node_range()) {
      const CFG::Node& node = cfg.lookup(ref);
      std::visit(util::overloaded {
            [] (CFG::Entry) {},
            [] (CFG::Exit)  {},
            [&] (const llvm::Instruction *I) {
               if (I->getType()->isPointerTy()) {
                  auto& pts = addrmap[I];
                  for (const auto& pair : addrmap) {
                     const llvm::Instruction *other = pair.first;
                     const llvm::AliasResult res = AA.alias(I, other);
                     switch (res) {
                     case llvm::MustAlias:
                        pts = addrmap[other];
                        break;
                     case llvm::MayAlias:
                     case llvm::PartialAlias: {
                        const auto& map = addrmap[other];
                        pts.insert(map.begin(), map.end());
                        break;
                     }
                     case llvm::NoAlias:
                     default:
                        break;
                     } 
                  }
                  pts.insert(next_id++);
               }
            }
         }, node.v);
   }

   for (NodeRef ref : node_range()) {
      const CFG::NodeRef cfg_ref = lookup(ref).cfg_ref;
      const CFG::Node& cfg_node = cfg.lookup(cfg_ref);
      if (const auto I = std::get_if<const llvm::Instruction *>(&cfg_node.v)) {
         Node& node = lookup(ref);
         const auto& pts = addrmap[*I];
         node.addrs.insert(pts.begin(), pts.end());
      }
   }
   
}
