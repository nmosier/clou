#include <optional>
#include <string>

#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <llvm/IR/Dominators.h>
#include <llvm/Analysis/LoopInfo.h>

#include <gperftools/profiler.h>

using llvm::errs;
   
struct LCMPass : public llvm::FunctionPass {
   static char ID;

   LCMPass(): FunctionPass(ID) {}
      
   virtual bool runOnFunction(llvm::Function& F) override {
      errs() << "In a function called " << F.getName() << "!\n";
         
      errs() << "Function body:\n";
      F.print(llvm::errs());


      const llvm::DominatorTree dom_tree {F};
      const llvm::LoopInfo loop_info {dom_tree};

      for (const llvm::Loop *loop : loop_info) {
         llvm::errs() << *loop << "\n";
      }

      return false;
   }
};

char LCMPass::ID = 0;

// Automatically enable the pass.
// http://adriansampson.net/blog/clangpass.html
static void registerLCMPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
   PM.add(new LCMPass());
}
static llvm::RegisterStandardPasses RegisterMyPass {
   llvm::PassManagerBuilder::EP_EarlyAsPossible,
   registerLCMPass
};
