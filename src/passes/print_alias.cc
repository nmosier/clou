#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/MemoryDependenceAnalysis.h>
#include <llvm/IR/CFG.h>
#include <llvm/Analysis/AliasAnalysis.h>

#include <unordered_set>

struct PrintAliasPass final: public llvm::FunctionPass {
    static inline char ID = 0;
    
    PrintAliasPass(): llvm::FunctionPass(ID) {}
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override {
        AU.addRequired<llvm::AAResultsWrapperPass>();
    }
    
    virtual bool runOnFunction(llvm::Function& F) override {
        llvm::AliasAnalysis& AA = getAnalysis<llvm::AAResultsWrapperPass>().getAAResults();
        
        // get all stores
        std::unordered_set<const llvm::StoreInst *> stores;
        for (const llvm::BasicBlock& B : F) {
            for (const llvm::Instruction& I : B) {
                if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(&I)) {
                    stores.insert(SI);
                }
            }
        }
        
        // print aliases
        llvm::outs() << F.getName() << ":\n";
        for (auto it1 = stores.begin(); it1 != stores.end(); ++it1) {
            const llvm::StoreInst *SI1 = *it1;
            for (auto it2 = std::next(it1); it2 != stores.end(); ++it2) {
                const llvm::StoreInst *SI2 = *it2;
                const llvm::AliasResult result = AA.alias(SI1->getPointerOperand(), SI2->getPointerOperand());
                llvm::outs() << *SI1 << " ~ " << *SI2 << " : ";
                switch (result) {
                    case llvm::NoAlias:
                        llvm::outs() << "no";
                        break;
                    case llvm::MayAlias:
                        llvm::outs() << "may";
                        break;
                    case llvm::MustAlias:
                        llvm::outs() << "must";
                        break;
                    default:
                        std::abort();
                }
                llvm::outs() << "\n";
            }
        }
        llvm::outs() << "\n\n\n";
        
        return false;
    }
};

namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new PrintAliasPass());
}

#if 1
llvm::RegisterStandardPasses registerCFP0 {
    llvm::PassManagerBuilder::EP_EarlyAsPossible,
    registerPass,
};
#endif

llvm::RegisterPass<PrintAliasPass> X {
    "print_alias", "Print Alias Pass"
};

}

