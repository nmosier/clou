#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/MemoryDependenceAnalysis.h>
#include <llvm/IR/CFG.h>
#include <llvm/Analysis/AliasAnalysis.h>

#include "dataflow/transient-aa.h"

struct PrintTransAA final: public llvm::FunctionPass {
    static inline char ID = 0;
    
    PrintTransAA(): llvm::FunctionPass(ID) {}
    

    virtual bool runOnFunction(llvm::Function& F) override {
        TransientAAResults transaa (F);
        
        return false;
    }
    
};

llvm::RegisterPass<PrintTransAA> X {
    "print_transaa", "Print Transient Alias Analysis"
};
