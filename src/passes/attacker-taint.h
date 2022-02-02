#pragma once

#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Function.h>

#include <cstdlib>

class AttackerTaintResults {
public:
    bool get(const llvm::Value *V) const;

private:
    /** Function that this analysis applies to. */
    const llvm::Function *F;
    
    /** Tainted instructions .*/
    std::set<const llvm::Instruction *> insts;
    
    friend struct AttackerTaintPass;
};

struct AttackerTaintPass final: public llvm::FunctionPass {
    static inline char ID = 0;
    AttackerTaintResults results;
    
    AttackerTaintPass(): llvm::FunctionPass(ID) {}
    
    AttackerTaintResults getResults() const { return results; }
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override;
    
    virtual bool runOnFunction(llvm::Function& F) override;
    
private:
    template <class Func>
    static void for_each_instruction(const llvm::Function& F, Func func);
};
