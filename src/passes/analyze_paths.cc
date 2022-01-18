#include <llvm/Pass.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>

struct AnalyzePathsPass final: public llvm::FunctionPass {
    static inline char ID = 0;
    
    unsigned count = 0;
    
    AnalyzePathsPass(): llvm::FunctionPass(ID) {}
    
    virtual bool runOnFunction(llvm::Function& F) override {
        
        // TODO
        
        return false;
    }
};

namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new AnalyzePathsPass());
}

llvm::RegisterStandardPasses registerAPP {
    llvm::PassManagerBuilder::EP_EarlyAsPossible,
    registerPass,
};

llvm::RegisterPass<AnalyzePathsPass> X {
    "analyze_paths", "Analyze Paths Pass",
};

}
