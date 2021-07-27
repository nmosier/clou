#pragma once

#include <string>
#include <cassert>

#include <llvm/IR/Instruction.h>
#include <z3++.h>

#include "graph.h"

class MemoryCFG;

class UHBContext {
public:
   z3::context context;

   z3::expr make_bool() { return context.bool_const(std::to_string(id_++).c_str()); }
   
private:
   unsigned id_ = 0;
};

struct UHBNode {
   const llvm::Instruction *inst;

   bool operator==(const UHBNode& other) const { return inst == other.inst; }
   bool operator!=(const UHBNode& other) const { return !(*this == other); }
   bool operator<(const UHBNode& other) const { return inst < other.inst; }
};

namespace std {
   template <>
   struct hash<UHBNode> {
      size_t operator()(const UHBNode& node) const {
         return std::hash<const llvm::Instruction *>()(node.inst);
      }
   };
}

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
   
private:
   Kind kind_;
   z3::expr constraint_;
};

class AEG {
public:
   using graph_t = Graph<UHBNode, UHBEdge>;
   void construct(const MemoryCFG& mcfg); // TODO: remove this.
   const graph_t& graph() const { return graph_; }

   void construct_full(const llvm::Function& F);

private:
   Graph<UHBNode, UHBEdge> graph_;
   
   void construct(const MemoryCFG& mcfg, const llvm::Instruction *I); // TODO: remove this.
   void construct_branch(const llvm::Instruction *inst);
};
