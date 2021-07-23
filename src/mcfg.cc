#include <unordered_set>

#include <llvm/IR/Instructions.h>

#include "mcfg.h"

using llvm::errs;

void MemoryCFG::build(const llvm::Function& F) {
   fwd.clear();
   successor_map(F, fwd);
   rev.clear();
   predecessor_map(F, rev);

   check(F);

   const auto *entry= &F.front().front();
   fwd[nullptr].insert(entry);
   rev[entry].insert(nullptr);

   check(F);
   
   prune(F);
}

void MemoryCFG::check(const llvm::Function& F) const {
   for (const auto& B : F) {
      for (const auto& I : B) {
         if (const auto *ret = llvm::dyn_cast<llvm::ReturnInst>(&I)) {
            assert(fwd.find(ret) == fwd.end());
         }
      }
   }   
}

/* 
 * 1. Create successor/predecessor map.
 * 2. 
 */

void MemoryCFG::prune(const llvm::Function& F) {
   for (const auto& B : F) {
      for (const auto& I : B) {
         check(F);
         const llvm::Instruction *inst = &I;
         switch (I.getOpcode()) {
         case llvm::Instruction::Ret: {
            const auto rev_it = rev.find(inst);
            if (rev_it != rev.end()) {
               for (const auto *pred : rev_it->second) {
                  auto& set = fwd.at(pred);
                  set.erase(inst);
                  set.insert(nullptr);
               }
               rev[nullptr].insert(rev_it->second.begin(), rev_it->second.end());
               rev.erase(rev_it);
            }
            break;
         }

         case llvm::Instruction::Load:
         case llvm::Instruction::Store:
         case llvm::Instruction::Fence:
            break;

         default:
            remove_node(inst);
            break;
         }
      }
   }
   check(F);
}


#if 0
void MemoryCFG::replace_node(const llvm::Instruction *old_inst, const llvm::Instruction *new_inst) {
   const auto fwd_it = fwd.find(old_inst);
   fwd.erase(fwd_it);
   
   fwd.at(old_inst)
}
#endif

void MemoryCFG::remove_node(const llvm::Instruction *inst) {
   const auto fwd_it = fwd.find(inst);
   const auto rev_it = rev.find(inst);

   /* add cross-product of edges to both */
   for (const llvm::Instruction *fwd_inst : fwd_it->second) {
      for (const llvm::Instruction *rev_inst : rev_it->second) {
         fwd[rev_inst].insert(fwd_inst);
         rev[fwd_inst].insert(rev_inst);
      }
   }

   for (const llvm::Instruction *fwd_inst : fwd_it->second) {
      rev.at(fwd_inst).erase(inst);
   }

   for (const llvm::Instruction *rev_inst : rev_it->second) {
      fwd.at(rev_inst).erase(inst);
   }

   fwd.erase(fwd_it);
   rev.erase(rev_it);
}
