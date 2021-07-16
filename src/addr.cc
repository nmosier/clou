#include "addr.h"

void AddressDependencyAnalysis::transfer(const llvm::Instruction *inst, const Value& in, Value& out)
{
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

   
void AddressDependencyAnalysis::meet(const Value& a, const Value& b, Value& res) {
   res = a;
   for (const auto& pair : b) {
      res[pair.first].insert(pair.second.begin(), pair.second.end());
   }
}
