#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>

#include "z3-util.h"
#include "aeg.h"
#include "mcfg.h"
#include "aeg-po.h"

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
   cfg_ref(ref), po(c.make_bool()), tfo(c.make_bool()) {}

void AEG::construct(const AEGPO& po, unsigned spec_depth) {
   // initialize nodes
   std::transform(po.nodes.begin(), po.nodes.end(), std::back_inserter(nodes),
                  [&] (const AEGPO::Node& node) {
                     return Node {node.cfg_ref, context};
                  });
   
   construct_nodes_po(po);
   construct_nodes_tfo(po, spec_depth);
   
}

void AEG::construct_nodes_po(const AEGPO& po) {
   for (NodeRef ref : node_range()) {
      const auto& preds = po.po.rev.at(ref);
      const auto& succs = po.po.fwd.at(ref);
      const Node& node = lookup(ref);

      /* add po constraint: exactly one successor */
      if (preds.empty()) {
         constraints &= node.po;
      } else if (!succs.empty()) {
         constraints &= util::one_of(succs.begin(), succs.end(), [&] (NodeRef dstref) {
            const Node& dst = lookup(dstref);
            return z3::implies(node.po, dst.po);
         }, context.TRUE, context.FALSE);
      }
   }
}

void AEG::construct_nodes_tfo(const AEGPO& po, unsigned spec_depth) {
   std::unordered_set<NodeRef> seen;
   std::deque<NodeRef> todo {po.entry};

   while (!todo.empty()) {
      const NodeRef noderef = todo.front();
      todo.pop_front();
      const auto& preds = po.po.rev.at(noderef);
      const auto& succs = po.po.fwd.at(noderef);
      std::copy(succs.begin(), succs.end(), std::back_inserter(todo));

      /* set tfo_depth */
      Node& node = lookup(noderef);
      if (preds.size() != 1) {
         node.tfo_depth = 0;
         constraints &= !node.tfo; // force TFO to false
      } else {
         const Node& pred = lookup(*preds.begin());
         node.tfo_depth = pred.tfo_depth + 1;
         if (node.tfo_depth > spec_depth) {
            constraints &= !node.tfo;
         } else {
            constraints &= z3::implies(node.tfo, pred.tfo);
         }
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
         << "tfo_depth: " << node.tfo_depth << "\n";
      
      dot::emit_kvs(os, "label", ss.str());
      os << ";\n";
   }

   // define edges
   for (const auto& pair : graph.fwd) {
      NodeRef src = pair.first;
      os << names.at(src) << " -> {";
      for (const auto& pair2 : pair.second) {
         NodeRef dst = pair2.first;
         // const Edge& edge = pair2.second;
         os << names.at(dst) << " ";
      }
      os << "}\n";
   }

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
   constraints = constraints.simplify();
}
