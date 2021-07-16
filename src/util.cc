#include <sstream>

#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/AliasAnalysis.h>


#include "util.h"

void predecessor_map(const llvm::Function& F, PredecessorMap& preds) {
   for (const llvm::BasicBlock& B : F) {
      for (auto cur = B.begin(), next = std::next(cur); next != B.end(); cur = next, ++next) {
         preds[&*next].push_back(&*cur);
      }

      /* successors */
      const llvm::Instruction *terminator = &B.back();
      for (size_t i = 0; i < terminator->getNumSuccessors(); ++i) {
         const llvm::BasicBlock *successor = terminator->getSuccessor(i);
         preds[&successor->front()].push_back(terminator);
      }
   }
   
}
