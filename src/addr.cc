#include <llvm/IR/Instructions.h>

#include "addr.h"

void AddressDependencyAnalysis::transfer(const llvm::Instruction *inst,
                                         const Value& in, Value& out) const {
   /* If it's a load instruction, then we kill the input set and make the single output the result
    * of the load (i.e. the instruction).
    * Otherwise, we check whether any operands use a value in the input set. If so, we add the result
    * to the set.
    * 
    * We also need to be able to remember the load instructions that the stores will eventually
    * source.
    */

   out = in;

   const auto opcode = inst->getOpcode();
   const bool isload = opcode == llvm::Instruction::Load;

   /* Check operands */
   for (const llvm::Use& operand : inst->operands()) {
      const llvm::Value *value = operand.get();
      if (const llvm::Instruction *src_inst = llvm::dyn_cast<llvm::Instruction>(value)) {
         // TODO: This could be optimized.
         /* iterate through all taint sets */
         for (auto& pair : out) {
            auto& taint_set = pair.second;
            if (taint_set.find(src_inst) != taint_set.end()) {
               /* add new instruction to taint set */
               taint_set.insert(inst);
            }
         }
      }
   }
   
   /* If load, then add */
   if (isload) {
      out[inst] = {inst};
   }
}


void AddressDependencyAnalysis::meet(const Value& a, const Value& b, Value& res) const {
   res = a;
   for (const auto& pair : b) {
      res[pair.first].insert(pair.second.begin(), pair.second.end());
   }
}


void AddressDependencyAnalysis::getResult(const llvm::Function& F, BinaryInstRel& addr) const {
   for (const auto& B : F) {
      for (const auto& I : B) {
         if (const auto *dst = llvm::dyn_cast<llvm::LoadInst>(&I)) {
            const auto& out = get_out(*dst);
            for (const auto& pair : out) {
               const auto *src = pair.first;
               const auto& taint = pair.second;
               if (src == dst) {
                  /* check if any of dst's operands are in taint set */
                  for (const auto& U : dst->operands()) {
                     if (const auto *inst_op = llvm::dyn_cast<llvm::Instruction>(U.get())) {
                        if (taint.find(inst_op) != taint.end()) {
                           addr[src].insert(dst);
                           break;
                        }
                     }
                  }
                     
               } else {
                  if (taint.find(dst) != taint.end()) {
                     addr[src].insert(dst);
                  }
               }
            }
         }
      }
   }   
}

