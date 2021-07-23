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

   z3::expr make_bool() { return context.bool_const(id_++); }
   
private:
   unsigned id_ = 0;
};

using UHBNode = const llvm::Instruction *;

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
   void construct(const MemoryCFG& mcfg);
   const graph_t& graph() const { return graph_; }

private:
   Graph<UHBNode, UHBEdge> graph_;
   
   void construct(const MemoryCFG& mcfg, const llvm::Instruction *I);
};
