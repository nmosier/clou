#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/MemoryDependenceAnalysis.h>
#include <llvm/IR/CFG.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <llvm/Analysis/LoopInfo.h>
#include <llvm/Analysis/LoopPass.h>
#include <llvm/IR/IntrinsicInst.h>
#include <llvm/IR/Operator.h>
#include <llvm/IR/Intrinsics.h>

struct PrintLoopsPass final: public llvm::LoopPass {
    static inline char ID = 0;
    
    PrintLoopsPass(): llvm::LoopPass(ID) {}
    
    virtual bool runOnLoop(llvm::Loop *L, llvm::LPPassManager& LPM) override {
        const auto& loc_range = L->getLocRange();
        
        llvm::errs() << "Loop (" << loc_range.getStart().getLine() << "-" << loc_range.getEnd().getLine() << "):\n";
        
        // check for min iteration annotation
        for (const llvm::BasicBlock *B : L->blocks()) {
            for (const llvm::Instruction& I : *B) {
                if (const llvm::IntrinsicInst *II = llvm::dyn_cast<llvm::IntrinsicInst>(&I)) {
                    if (II->getIntrinsicID() == llvm::Intrinsic::var_annotation) {
                        
                        const llvm::Value *V = llvm::cast<llvm::GEPOperator>(II->getArgOperand(1))->getPointerOperand();
                        llvm::errs() << *V << "\n";
                        const llvm::Constant *C = llvm::cast<llvm::Constant>(V);
                        const llvm::GlobalValue *GV = llvm::cast<llvm::GlobalValue>(C);
                        const llvm::GlobalVariable *GV_ = llvm::cast<llvm::GlobalVariable>(GV);
                        const llvm::Constant *init_C = GV_->getInitializer();
                        llvm::errs() << *init_C << "\n";
                        const auto *init_CA = llvm::cast<llvm::ConstantDataArray>(init_C);
                        llvm::errs() << init_CA->getAsCString() << "\n";
                    }
                }
            }
        }
        
        
        
        return false;
        
    }
    
};

namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new PrintLoopsPass());
}

#if 1
llvm::RegisterStandardPasses registerCFP0 {
    llvm::PassManagerBuilder::EP_EarlyAsPossible,
    registerPass,
};
#endif

llvm::RegisterPass<PrintLoopsPass> X {
    "print_loops", "Print Loops Pass"
};

}
