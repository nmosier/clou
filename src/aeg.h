#pragma once

#include <string>
#include <cassert>
#include <deque>

#include <llvm/IR/Instruction.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <z3++.h>

#include "z3-util.h"
#include "aeg-po2.h"
#include "graph.h"
#include "inst.h"
#include "util.h"

class UHBContext {
public:
   UHBContext(): context(), TRUE(context.bool_val(true)), FALSE(context.bool_val(false)) {}
   
   z3::context context;

   z3::expr make_bool() { return context.bool_const(std::to_string(id_++).c_str()); }
   z3::expr make_int() { return context.int_const(std::to_string(id_++).c_str()); }

   const z3::expr TRUE;
   const z3::expr FALSE;

private:
   unsigned id_ = 0;
};

struct UHBConstraints {
   const z3::expr TRUE;
   std::vector<z3::expr> exprs;

   UHBConstraints(const UHBContext& ctx): TRUE(ctx.TRUE) {}
   
   void add_to(z3::solver& solver) const {
      for (const z3::expr& expr : exprs) {
         solver.add(expr, util::to_string(expr).c_str());
      }
   }
   void operator()(const z3::expr& clause) {
      exprs.push_back(clause);
   }
   void simplify() {
      std::for_each(exprs.begin(), exprs.end(), [] (z3::expr& e) {
         e = e.simplify();
      });
   }
};

inline std::ostream& operator<<(std::ostream& os, const UHBConstraints& c) {
   return os <<
      std::reduce(c.exprs.begin(), c.exprs.end(), c.TRUE,
                  [] (const z3::expr& a, const z3::expr& b) {
                     return a && b;
                  }); // .simplify();
}


struct UHBAddress {
   const llvm::Value *value;
   z3::expr class_id;
   z3::expr instance_id;
};

struct UHBNode {
   Inst inst;
   z3::expr po;  // program order variable
   z3::expr tfo; // transient fetch order variable
   z3::expr tfo_depth; // transient depth
   std::optional<UHBAddress> addr_def;
   std::unordered_map<const llvm::Value *, z3::expr> addr_refs;
   UHBConstraints constraints;
   
   void simplify() {
      po = po.simplify();
      tfo = tfo.simplify();
      tfo_depth = tfo_depth.simplify();
      constraints.simplify();
   }

   UHBNode(const Inst& inst, UHBContext& c);
};

struct UHBEdge {
#define UHBEDGE_KIND_X(X)                       \
   X(FORK)                                      \
   X(PO)                                        \
   X(TFO)                                       \
   X(RF)                                        \
   X(CO)                                        \
   X(FR)                                        \
   X(RFX)                                       \
   X(COX)                                       \
   X(FRX)                                       

#define UHBEDGE_KIND_E(name) name,
   enum Kind {
      UHBEDGE_KIND_X(UHBEDGE_KIND_E)
   };
#undef UHBEDGE_KIND_E

   Kind kind;
   z3::expr exists;
   UHBConstraints constraints;

   static const char *kind_tostr(Kind kind);
   const char *kind_tostr() const { return kind_tostr(kind); }
   static Kind kind_fromstr(const std::string& s);

   UHBEdge(Kind kind, UHBContext& ctx):
      kind(kind), exists(ctx.make_bool()), constraints(ctx) {} 

   struct Hash {
      size_t operator()(const UHBEdge& x) const { return std::hash<Kind>()(x.kind); }
   };

   bool operator==(const UHBEdge& other) const { return kind == other.kind; }
   void simplify() { constraints.simplify(); }
};

std::ostream& operator<<(std::ostream& os, const UHBEdge& e);

class AEG {
public:
   using Node = UHBNode;
   using NodeRef = std::size_t;
   using Edge = UHBEdge;
   using graph_type = Graph<NodeRef, Edge, std::hash<NodeRef>, Edge::Hash>;

   static inline const NodeRef entry {0};

   graph_type graph;

   template <typename T>
   void construct(const AEGPO_Base<T>& po, unsigned spec_depth, llvm::AliasAnalysis& AA);

   const Node& lookup(NodeRef ref) const { return nodes.at(static_cast<unsigned>(ref)); }
   Node& lookup(NodeRef ref) { return nodes.at(static_cast<unsigned>(ref)); }

   AEG(): context(), constraints(context) {}

   void dump_graph(llvm::raw_ostream& os) const;
   void dump_graph(const std::string& path) const;

   void simplify();

   void test();
   
private:
   UHBContext context;
   UHBConstraints constraints;
   std::vector<Node> nodes;

   template <typename T>
   void construct_nodes_po(const AEGPO_Base<T>& po);

   template <typename T>
   void construct_nodes_tfo(const AEGPO_Base<T>& po, unsigned spec_depth);

   template <typename T>
   void construct_edges_po_tfo(const AEGPO_Base<T>& po);

   template <typename T>
   void construct_aliases(const AEGPO_Base<T>& po, llvm::AliasAnalysis& AA);
   
   using NodeRange = util::RangeContainer<NodeRef>;
   NodeRange node_range() const {
      return NodeRange {NodeRef {entry}, NodeRef {static_cast<unsigned>(nodes.size())}};
   }
   
#if 0
   template <typename... Ts>
   NodeRef add_node(Ts&&... ts) {
      const NodeRef ref {static_cast<unsigned>(nodes.size())};
      nodes.emplace_back(std::forward<Ts>(ts)...);
      return ref;
   }
#endif

   NodeRef find_upstream_def(NodeRef node, const llvm::Value *addr_ref) const;
};


template <typename T>
void AEG::construct(const AEGPO_Base<T>& po, unsigned spec_depth, llvm::AliasAnalysis& AA) {
   // initialize nodes
   std::transform(po.nodes.begin(), po.nodes.end(), std::back_inserter(nodes),
                  [&] (const auto& node) {
                     const Inst inst =
                        std::visit(util::creator<Inst>(),
                                   node());
                     return Node {inst, context};
                  });
   for (NodeRef ref : node_range()) {
      graph.add_node(ref);
   }

   if (verbose >= 2) { llvm::errs() << "Constructing nodes po\n"; }
   construct_nodes_po(po);
   if (verbose >= 2) { llvm::errs() << "Constructing nodes tfo\n"; }   
   construct_nodes_tfo(po, spec_depth);
   if (verbose >= 2) { llvm::errs() << "Constructing edges po tfo\n"; }
   construct_edges_po_tfo(po);
   if (verbose >= 2) { llvm::errs() << "Constructing aliases\n"; }
   construct_aliases(po, AA);
}

template <typename T>
void AEG::construct_nodes_po(const AEGPO_Base<T>& po) {
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

template <typename T>
void AEG::construct_nodes_tfo(const AEGPO_Base<T>& po, unsigned spec_depth) {
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

template <typename T>
void AEG::construct_edges_po_tfo(const AEGPO_Base<T>& po) {
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

template <typename T>
void AEG::construct_aliases(const AEGPO_Base<T>& po, llvm::AliasAnalysis& AA) {
   
   
}
