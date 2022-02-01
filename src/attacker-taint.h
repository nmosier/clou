#pragma once

#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Function.h>

#include <cstdlib>

class AttackerTaintResults {
public:
    bool get(const llvm::Value *V) const {
        if (const llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(V)) {
            if (I->getFunction() == F) {
                return insts.contains(I);
            } else {
                return true;
            }
        } else if (llvm::isa<llvm::Argument>(V)) {
            return true;
        } else if (llvm::isa<llvm::Constant, llvm::BasicBlock>(V)) {
            return false;
        } else {
            llvm::errs() << "unhandled value: " << *V << "\n";
            std::abort();
        }
    }

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
};
