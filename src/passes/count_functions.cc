#include <llvm/Pass.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>

struct CountFunctionsPass final: public llvm::ModulePass {
    static inline char ID = 0;
    
    unsigned count = 0;
    
    CountFunctionsPass(): llvm::ModulePass(ID) {}
    
    virtual bool runOnModule(llvm::Module& M) override {
        for (llvm::Function& F : M) {
            if (!F.isDeclaration()) {
                ++count;
            }
        }
        print(llvm::errs(), &M);
        return false;
    }
    
    virtual void print(llvm::raw_ostream& os, const llvm::Module *M) const override {
        os << count << "\n";
    }
};


namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new CountFunctionsPass());
}

#if 1
llvm::RegisterStandardPasses registerCFP {
    llvm::PassManagerBuilder::EP_EnabledOnOptLevel0,
    registerPass,
};
#endif

llvm::RegisterStandardPasses registerCFP0 {
    llvm::PassManagerBuilder::EP_ModuleOptimizerEarly,
    registerPass,
};

llvm::RegisterStandardPasses registerCFP1 {
    llvm::PassManagerBuilder::EP_FullLinkTimeOptimizationEarly,
    registerPass,
};

llvm::RegisterPass<CountFunctionsPass> X {
    "count_functions", "Count Functions Pass",
};

}
