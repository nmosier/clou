#pragma once

#include "util.h"

class MemoryCFG {
public:
   MemoryCFG(const llvm::Function& F) { build(F); }

   void build(const llvm::Function& F);

   const BinaryInstRel& graph(bool reverse = false) const {
      return reverse ? rev : fwd;
   }

private:
   BinaryInstRel fwd;
   BinaryInstRel rev;
   const llvm::Instruction *entry_;

   void prune(const llvm::Function& F);
   void remove_node(const llvm::Instruction *inst);
   void check(const llvm::Function& F) const;
};
