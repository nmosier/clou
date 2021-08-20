#pragma once

#include <string>
#include <cassert>
#include <unordered_set>

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
   std::optional<z3::expr> addr_def;
   std::vector<std::pair<const llvm::Value *, z3::expr>> addr_refs;
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

   void construct(unsigned spec_depth, llvm::AliasAnalysis& AA);

   const Node& lookup(NodeRef ref) const { return nodes.at(static_cast<unsigned>(ref)); }
   Node& lookup(NodeRef ref) { return nodes.at(static_cast<unsigned>(ref)); }

   explicit AEG(const AEGPO2& po): po(po), context(), constraints(context) {}

   void dump_graph(llvm::raw_ostream& os) const;
   void dump_graph(const std::string& path) const;

   void simplify();

   void test();

   std::size_t size() const { return nodes.size(); }

private:
   const AEGPO2& po;
   UHBContext context;
   UHBConstraints constraints;
   std::vector<Node> nodes;

   void construct_nodes();
   void construct_nodes_po();
   void construct_nodes_tfo(unsigned spec_depth);
   void construct_edges_po_tfo();
   void construct_nodes_addr_defs();
   void construct_nodes_addr_refs();
   void construct_aliases(llvm::AliasAnalysis& AA);
   void construct_com();

   struct CondNode {
      NodeRef ref;
      z3::expr cond;
   }; 
   template <Inst::Kind KIND, typename OutputIt>
   void find_sourced_memops(NodeRef org, OutputIt out) const;
   template <typename OutputIt>
   void find_sourced_writes(NodeRef read, OutputIt out) const;
   template <typename OutputIt>
   void find_sourced_reads(NodeRef read, OutputIt out) const;
   template <typename OutputIt>
   void find_preceding_writes(NodeRef write, OutputIt out) const;
   

   using NodeRange = util::RangeContainer<NodeRef>;
   NodeRange node_range() const {
      return NodeRange {NodeRef {entry}, NodeRef {static_cast<unsigned>(nodes.size())}};
   }

   void find_upstream_def(NodeRef node, const llvm::Value *addr_ref,
                          std::unordered_set<NodeRef>& out) const;

   NodeRef add_node(const Node& node) {
      const NodeRef ref = size();
      nodes.push_back(node);
      graph.add_node(ref);
      return ref;
   }
};


