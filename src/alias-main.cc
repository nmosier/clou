#include <optional>
#include <string>

#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/AliasAnalysis.h>

#include <gperftools/profiler.h>

#include "util.h"
#include "lcm.h"
#include "addr.h"
#include "mcfg.h"
#include "aeg-po.h"
#include "config.h"
#include "cfg.h"
#include "aeg.h"

using llvm::errs;

   
struct LCMPass : public llvm::FunctionPass {
   static char ID;

   LCMPass(): FunctionPass(ID) {}

   virtual void getAnalysisUsage(llvm::AnalysisUsage& usage) const override {
      usage.addRequired<llvm::AAResultsWrapperPass>();
   }
      
   virtual bool runOnFunction(llvm::Function& F) override {
      llvm::AliasAnalysis& AA = getAnalysis<llvm::AAResultsWrapperPass>().getAAResults();

      errs() << "In a function called " << F.getName() << "!\n";
         
      errs() << "Function body:\n";
      F.print(llvm::errs());

      /* Gather all pointers */
      std::vector<const llvm::Instruction *> pointers;
      for (const auto& B : F) {
         for (const auto& I : B) {
            if (I.getType()->isPointerTy()) {
               pointers.push_back(&I);
            }
         }
      }

      for (auto it1 = pointers.begin(); it1 != pointers.end(); ++it1) {
         for (auto it2 = std::next(it1, 1); it2 != pointers.end(); ++it2) {
            const llvm::AliasResult res = AA.alias(*it1, *it2);
            static const std::unordered_map<llvm::AliasResult, const char *> strmap {
               {llvm::NoAlias, "no alias"},
               {llvm::MayAlias, "may alias"},
               {llvm::PartialAlias, "partial alias"},
               {llvm::MustAlias, "must alias"},
            };
            errs() << **it1 << "\t" << **it2 << "\t"
                   << strmap.at(static_cast<llvm::AliasResult>(res)) << "\n";
         }
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
