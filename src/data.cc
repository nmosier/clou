#include "data.h"

/* DATA DEPENDENCY ANALYSIS 
 * Values map a destination register to the set of the registers it depends on.
 */
void DataDependencyAnalysis::transfer(const llvm::Instruction *inst,
                                      const Value& in, Value& out) const {
   out = in;

   const auto opcode = inst->getOpcode();

   /* Mark this instruction as self-dependent. */
   out[inst] = {inst};

   /* Get set of this instruction's dependencies */
   std::unordered_set<const llvm::Instruction *> deps;
   for (const llvm::Use& use : inst->operands()) {
      if (const llvm::Instruction *src_inst = llvm::dyn_cast<llvm::Instruction>(use.get())) {
         deps.insert(src_inst);
      }
   }
   
   /* For all instructions, propogate dependents if they depend on this instruction. */
   for (auto& pair : out) {
      auto& src_set = pair.second;
      if (src_set.find(inst) != src_set.end()) {
         src_set.insert(deps.begin(), deps.end());
      }
   }
   
   for (const llvm::Use& operand : inst->operands()) {
      auto& depset = out[inst];
      const llvm::Value *value = operand.get();
      if (const llvm::Instruction *src_inst = llvm::dyn_cast<llvm::Instruction>(value)) {
         /* add to current instruction's dependency set */
         depset.insert(src_inst);
      }
   }
}


void DataDependencyAnalysis::meet(const Value& a, const Value& b, Value& res) const {
   res = a;
   for (const auto& pair : b) {
      res[pair.first].insert(pair.second.begin(), pair.second.end());
   }
}

void DataDependencyAnalysis::getResult(const llvm::Function& F, BinaryInstRel& addr) const {
   /* Theorem: the dependencies at program entry is an underapproximation of the dependencies of the 
    * program.
    * Note that this loses information about 
    */
   
   /* For each instruction, take the union of all dependencies over all program points to
    * get the complete dependency set. 
    */

   
   
   for (const auto& B : F) {
      for (const auto& I : B) {
         const auto out = get_out(I);
         const auto& deps = 
      }
   }
}
