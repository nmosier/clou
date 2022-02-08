#pragma once

#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Function.h>

#include <cstdlib>
#include <set>

class AttackerTaintResults {
public:
    bool get(const llvm::Value *V) const;

private:
    /** Function that this analysis applies to. */
    const llvm::Function *F;
    
    /** Tainted instructions .*/
    std::set<const llvm::Instruction *> insts;
    
    friend struct AttackerTaintPass;
    friend llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AttackerTaintResults&);
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AttackerTaintResults&);



struct AttackerTaintPass final: public llvm::FunctionPass {
    static inline char ID = 0;
    AttackerTaintResults results;
    
    AttackerTaintPass(): llvm::FunctionPass(ID) {}
    
    AttackerTaintResults getResults() const { return results; }
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override;
    
    virtual bool runOnFunction(llvm::Function& F) override;
    
    virtual void print(llvm::raw_ostream& os, const llvm::Module *M) const override;
    
private:
    template <class Func>
    static void for_each_instruction(const llvm::Function& F, Func func);
};
