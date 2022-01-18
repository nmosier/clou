#include <cassert>

#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>

struct PrintLinkagePass final: public llvm::FunctionPass {
    static inline char ID = 0;
    
    PrintLinkagePass(): llvm::FunctionPass(ID) {}
    
    virtual bool runOnFunction(llvm::Function& F) override {
        assert(!F.isDeclaration());
        
#if 0
        print_func(&F);
#endif
        
        for (const llvm::BasicBlock& BB : F) {
            for (const llvm::Instruction& I : BB) {
                if (const llvm::CallBase *C = llvm::dyn_cast<llvm::CallBase>(&I)) {
                    const llvm::Function *F = C->getCalledFunction();
                    print_func(F);
                }
            }
        }

        return false;
    }
    
    static void print_func(const llvm::Function *F) {
        if (F) {
            llvm::errs() << F->getName() << " ";
            if (F->isDeclaration()) {
                llvm::errs() << "declaration";
            } else {
                llvm::errs() << "definition";
            }
            llvm::errs() << "\n";
        }
    }
};



namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new PrintLinkagePass());
}

#if 0
llvm::RegisterStandardPasses registerCFP {
    llvm::PassManagerBuilder::EP_EarlyAsPossible,
    registerPass,
};
#endif

llvm::RegisterStandardPasses registerCFP0 {
#if 0
    llvm::PassManagerBuilder::EP_ModuleOptimizerEarly,
#else
    llvm::PassManagerBuilder::EP_FullLinkTimeOptimizationEarly,
#endif
    registerPass,
};

llvm::RegisterPass<PrintLinkagePass> X {
    "print_linkage", "Print Linkage Pass",
};

}
