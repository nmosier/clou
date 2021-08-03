#pragma once

#include <string>
#include <cassert>

#include <llvm/IR/Instruction.h>
#include <z3++.h>

#include "cfg.h"
#include "aeg-po.h"
#include "graph.h"

class UHBContext {
public:
   UHBContext(): context(), TRUE(context.bool_val(true)), FALSE(context.bool_val(false)) {}
   
   z3::context context;

   z3::expr make_bool() { return context.bool_const(std::to_string(id_++).c_str()); }

   const z3::expr TRUE;
   const z3::expr FALSE;
   
private:
   unsigned id_ = 0;
};

struct UHBNode {
   CFG::NodeRef cfg_ref;
   z3::expr po;  // program order constraint
   z3::expr tfo; // transient fetch order constraint
   unsigned tfo_depth; 

   bool operator==(const UHBNode& other) const { return cfg_ref == other.cfg_ref; }
   bool operator!=(const UHBNode& other) const { return !(*this == other); }

   struct Hash {
      size_t operator()(const UHBNode& node) const {
         return std::hash<CFG::NodeRef>()(node.cfg_ref);
      }
   };

   void simplify() {
      po.simplify();
      tfo.simplify();
   }

   UHBNode(CFG::NodeRef ref, UHBContext& c);
};

class UHBEdge {
public:
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

   Kind kind() const { return kind_; }
   static const char *kind_tostr(Kind kind);
   const char *kind_tostr() const { return kind_tostr(kind()); }
   static Kind kind_fromstr(const std::string& s);

   UHBEdge(Kind kind, UHBContext& context): kind_(kind), constraint_(context.context) {} 
   UHBEdge(Kind kind, const z3::expr& constraint): kind_(kind), constraint_(constraint) {
      assert(constraint.is_bool());
   }

   struct Hash {
      size_t operator()(const UHBEdge& x) const { return std::hash<Kind>()(x.kind_); }
   };
   
private:
   Kind kind_;
   z3::expr constraint_;
   friend class std::hash<UHBEdge>;
};

class AEG {
public:
   using Node = UHBNode;
   using NodeRef = AEGPO::NodeRef;
   using Edge = UHBEdge;
   using graph_type = Graph<NodeRef, Edge, std::hash<NodeRef>, Edge::Hash>;

   static inline const NodeRef entry {0};

   graph_type graph;
   
   void construct(const AEGPO& po, unsigned spec_depth);

   const Node& lookup(NodeRef ref) const { return nodes.at(static_cast<unsigned>(ref)); }
   Node& lookup(NodeRef ref) { return nodes.at(static_cast<unsigned>(ref)); }
   
   AEG(const CFG& cfg): cfg(cfg), context(), constraints(context.TRUE) {}

   void dump_graph(llvm::raw_ostream& os) const;
   void dump_graph(const std::string& path) const;

   void simplify();
   
private:
   const CFG& cfg;
   UHBContext context;
   z3::expr constraints;
   std::vector<Node> nodes;

   void construct_nodes_po(const AEGPO& po);
   void construct_nodes_tfo(const AEGPO& po, unsigned spec_depth);
   void construct_edges(); // TODO

   using NodeRange = util::RangeContainer<NodeRef>;
   NodeRange node_range() const {
      return NodeRange {NodeRef {entry}, NodeRef {static_cast<unsigned>(nodes.size())}}; }
   
   
#if 0
   template <typename... Ts>
   NodeRef add_node(Ts&&... ts) {
      const NodeRef ref {static_cast<unsigned>(nodes.size())};
      nodes.emplace_back(std::forward<Ts>(ts)...);
      return ref;
   }
#endif
};
