#pragma once

#include <iostream>

#include <llvm/Pass.h>
#include <llvm/Analysis/CallGraphSCCPass.h>
#include <llvm/Analysis/CallGraph.h>
#include <llvm/IR/Function.h>

struct FunctionOrderingPass: public llvm::CallGraphSCCPass {
    static inline char ID = 0;
    
    std::vector<llvm::Function *> order;
    
    FunctionOrderingPass(): llvm::CallGraphSCCPass(ID) {
        std::cerr << "HELLO\n";
    }
    ~FunctionOrderingPass() {
        std::cerr << "GOODBYE\n";
    }
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& usage) const override {
        usage.setPreservesAll();
    }
    
    virtual bool runOnSCC(llvm::CallGraphSCC& SCC) override {
        for (llvm::CallGraphNode *CGN : SCC) {
            const auto F = CGN->getFunction();
            if (F && !F->isDeclaration()) {
                order.push_back(F);
            }
            std::cerr << "here\n";
        }
        return false;
    }
    
    virtual void print(llvm::raw_ostream& os, const llvm::Module *M) const override {
        for (llvm::Function *F : order) {
            std::cerr << F->getName().str() << "\n";
        }
    }
};

#if 0
inline llvm::RegisterPass<FunctionOrderingPass> register_function_ordering {
    "funcorder", "Function Ordering Pass", false, true
};
#endif
