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
        locs.emplace(&F, F.getInstructionCount());
        return false;
    }
    
    virtual void print(llvm::raw_ostream& os, const llvm::Module *M) const override {
        for (const auto& p : locs) {
            os << p.first->getName() << " " << p.second << "\n";
        }
    }
};

namespace {

llvm::RegisterPass<LOCPass> X {
    "loc", "LOC Pass"
};

}
