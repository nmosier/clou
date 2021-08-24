#pragma once

#include <string>
#include <cassert>
#include <unordered_set>
#include <sstream>

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

   z3::expr to_expr(bool b) const {
      return b ? TRUE : FALSE;
   }

private:
   unsigned id_ = 0;
};

struct UHBConstraints {
   std::vector<z3::expr> exprs;

   UHBConstraints() {}
   explicit UHBConstraints(const z3::expr& expr): exprs({expr}) {}

   void add_to(z3::solver& solver) const {
      for (const z3::expr& expr : exprs) {
         std::stringstream ss;
         ss << expr << ":" << this;
         solver.add(expr, ss.str().c_str());
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

std::ostream& operator<<(std::ostream& os, const UHBConstraints& c);

struct UHBAddress {
   z3::expr po;
   z3::expr tfo;
   UHBAddress(UHBContext& ctx): po(ctx.make_int()), tfo(ctx.make_int()) {}
};

inline std::ostream& operator<<(std::ostream& os, const UHBAddress& x) {
   return os << "(po)" << x.po << " (tfo)" << x.tfo;
}

struct UHBNode {
   Inst inst;
   z3::expr po;  // program order variable
   z3::expr tfo; // transient fetch order variable
   z3::expr tfo_depth; // transient depth
   std::optional<UHBAddress> addr_def;
   std::vector<std::pair<const llvm::Value *, UHBAddress>> addr_refs;
   z3::expr xsread;
   z3::expr xswrite;
   UHBConstraints constraints;

   z3::expr get_addr_def() const {
      // TODO: Do we need to add an extra ite to qualify based on tfo bool too?
      return z3::ite(po, addr_def->po, addr_def->tfo);
   }

   z3::expr get_addr_ref(std::size_t idx) const {
      const UHBAddress& addr = addr_refs.at(idx).second;
      return z3::ite(po, addr.po, addr.tfo);
   }

   z3::expr get_exec() const {
      return po || tfo;
   }
   
   void simplify() {
      po = po.simplify();
      tfo = tfo.simplify();
      tfo_depth = tfo_depth.simplify();
      constraints.simplify();
   }

   z3::expr same_xstate(const UHBNode& other) const {
      return get_addr_ref(0) == other.get_addr_ref(0);
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
      kind(kind), exists(ctx.make_bool()) {}

   UHBEdge(Kind kind, const z3::expr& exists): kind(kind), exists(exists) {}

   struct Hash {
      size_t operator()(const UHBEdge& x) const { return std::hash<Kind>()(x.kind); }
   };

   bool operator==(const UHBEdge& other) const { return kind == other.kind; }
   void simplify() { constraints.simplify(); }
};

std::ostream& operator<<(std::ostream& os, const UHBEdge& e);
inline std::ostream& operator<<(std::ostream& os, UHBEdge::Kind kind) {
   return os << UHBEdge::kind_tostr(kind);
}

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

   explicit AEG(const AEGPO2& po): po(po), context(), constraints() {}

   void dump_graph(llvm::raw_ostream& os) const;
   void dump_graph(const std::string& path) const;

   void simplify();

   void test();

   std::size_t size() const { return nodes.size(); }

   using Path = std::vector<NodeRef>;

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
   void construct_comx();

   struct CondNode {
      NodeRef ref;
      z3::expr cond;
   }; 
   template <typename OutputIt>
   void find_sourced_memops(Inst::Kind kind, NodeRef org, OutputIt out) const;
   template <typename OutputIt>
   void find_preceding_memops(Inst::Kind kind, NodeRef write, OutputIt out) const;
   

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

   template <typename OutputIt>
   void find_comx_window(NodeRef ref, unsigned distance, unsigned spec_depth, OutputIt out) const;

   void add_unidir_edge(NodeRef src, NodeRef dst, const UHBEdge& e) {
      graph.insert(src, dst, e);
   }

   void add_bidir_edge(NodeRef a, NodeRef b, const UHBEdge& e);
   void add_optional_edge(NodeRef src, NodeRef dst, const UHBEdge& e);

   template <typename OutputIt>
   OutputIt get_incoming_edges(NodeRef dst, OutputIt out, UHBEdge::Kind kind) {
      for (const auto& p : graph.rev.at(dst)) {
         std::cerr << "constr ";
         out = std::copy_if(p.second.begin(), p.second.end(), out, [kind] (const auto& e) {
            std::cerr << " " << e->kind;
            return e->kind == kind;
         });
        std::cerr << "\n";
      }
      return out;
   }

   void output_execution(std::ostream& os, const z3::model& model) const;
   void output_execution(const std::string& path, const z3::model& model) const;

   z3::expr check_no_intervening_writes(NodeRef src, NodeRef dst) const;
   // z3::expr check_no_intervening_writes_rec(NodeRef src, NodeRef

   bool is_ancestor(NodeRef parent, NodeRef child) const;
   bool is_ancestor_a(NodeRef, NodeRef) const;
   bool is_ancestor_b(NodeRef, NodeRef) const;
   
};


