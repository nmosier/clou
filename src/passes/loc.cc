#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/MemoryDependenceAnalysis.h>
#include <llvm/IR/CFG.h>
#include <llvm/Analysis/AliasAnalysis.h>

#include <unordered_map>

struct LOCPass final: public llvm::FunctionPass {
    static inline char ID = 0;
    
    LOCPass(): llvm::FunctionPass(ID) {}
    
    std::unordered_map<const llvm::Function *, unsigned> locs;
    
    virtual bool runOnFunction(llvm::Function& F) override {
        const unsigned instruction_count = F.getInstructionCount();
        
        // get LOC: find span
        unsigned max_source = 0;
        unsigned min_source = std::numeric_limits<unsigned>::max();
        
        
        
        const auto& p = *locs.emplace(&F, F.getInstructionCount()).first;
        llvm::errs() << p.first->getName() << " " << p.second << "\n";
        return false;
    }
    
    virtual void print(llvm::raw_ostream& os, const llvm::Module *M) const override {
        for (const auto& p : locs) {
            os << p.first->getName() << " " << p.second << "\n";
        }
    }
};

namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new LOCPass());
}

llvm::RegisterStandardPasses registerLOC {
    llvm::PassManagerBuilder::EP_EarlyAsPossible,
    registerPass,
};

llvm::RegisterPass<LOCPass> X {
    "loc", "LOC Pass"
};

}
