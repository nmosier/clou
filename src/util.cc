#include <sstream>

#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/AliasAnalysis.h>


#include "util.h"

void predecessor_map(const llvm::Function& F, BinaryInstRel& preds) {
   for (const llvm::BasicBlock& B : F) {
      for (auto cur = B.begin(), next = std::next(cur); next != B.end(); cur = next, ++next) {
         preds[&*next].insert(&*cur);
      }

      /* successors */
      const llvm::Instruction *terminator = &B.back();
      for (size_t i = 0; i < terminator->getNumSuccessors(); ++i) {
         const llvm::BasicBlock *successor = terminator->getSuccessor(i);
         preds[&successor->front()].insert(terminator);
      }
   }
   
}

void successor_map(const llvm::Function& F, BinaryInstRel& succs) {
   for (const llvm::BasicBlock& B : F) {
      for (auto cur = B.begin(), next = std::next(cur); next != B.end(); cur = next, ++next) {
         succs[&*cur].insert(&*next);
      }
      const llvm::Instruction *term = &B.back();
      const auto num_succs = term->getNumSuccessors();
      for (size_t i = 0; i < num_succs; ++i) {
         const llvm::Instruction *succ = &term->getSuccessor(i)->front();
         succs[term].insert(succ);
      }
   }
}


void get_cfg(const llvm::Function& F, CFG& cfg) {
   cfg.insert(nullptr, &F.front().front());
   for (const llvm::BasicBlock& B : F) {
      for (auto cur = B.begin(), next = std::next(cur); next != B.end(); cur = next, ++next) {
         cfg.insert(&*cur, &*next);
      }
      const llvm::Instruction *term = &B.back();
      const auto num_succs = term->getNumSuccessors();
      if (num_succs == 0) {
         cfg.insert(term, nullptr);
      } else {
         for (size_t i = 0; i < num_succs; ++i) {
            const llvm::Instruction *succ = &term->getSuccessor(i)->front();
            cfg.insert(term, succ);
         }
      }
   }
}
