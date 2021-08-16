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
#include "aeg-po2.h"

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



      /* Try dataflow analysis */
      AddressDependencyAnalysis addr_analysis;
      addr_analysis.run(F);
#if 0
      errs() << "\n\n\nADDRESS DEPENDENCY ANALYSIS\n";
      for (const auto& B : F) {
         for (const auto& I : B) {
            errs() << "Instruction: " << I << "\n";
            const auto& inval = addr_analysis.get_in(I);
            for (const auto& pair : inval) {
               if (!pair.second.empty()) {
                  errs() << "  " << *pair.first << "\n";
                  for (const llvm::Instruction *inst : pair.second) {
                     errs() << "      " << *inst << "\n";
                  }
               }
            }
            errs() << "\n";
         }
      }
#endif

      /* Construct addr relation */
      BinaryInstRel addr;
      addr_analysis.getResult(F, addr);

      errs() << "Address Relation:\n";
      for (const auto& pair : addr) {
         const auto *src = pair.first;
         for (const auto *dst : pair.second) {
            errs() << *src << "  ->  " << *dst << "\n";
         }
      }


      MemoryCFG mcfg {F};
      errs() << "\n\n\nMemory Control Flow Graph\n";
      for (const auto& src_pair : mcfg.graph()) {
         const auto *src = src_pair.first;
         for (const auto *dst : src_pair.second) {
            if (src) {
               errs() << *src;
            } else {
               errs() << "TOP";
            }
            errs() << "  ---->  ";
            if (dst) {
               errs() << *dst;
            } else {
               errs() << "BOT";
            }
            errs() << "\n";
         }
      }
      
      
      /* Construct AEG */

#if 0
      CFG cfg;
      cfg.construct(F);

      llvm::errs() << "CFG done\n";
      
      if (!cfg_output_path.empty()) {
         cfg.dump_graph(format_graph_path(cfg_output_path, F));
      }

      AEGPO aeg {cfg};
      
      
      ProfilerStart(format_graph_path("out/%s.prof", F).c_str());
      signal(SIGINT, [] (int sig) {
         ProfilerStop();
         std::exit(0);
      });
      aeg.construct2();
      ProfilerStop();

      if (!aegpo_output_path.empty()) {
         aeg.dump_graph(format_graph_path(aegpo_output_path, F));
      }

      AEG aeg2 {cfg};
      aeg2.construct(aeg, 2, AA);
      aeg2.simplify();

      if (!aeg_output_path.empty()) {
         aeg2.dump_graph(format_graph_path(aeg_output_path, F));
      }

      aeg2.test();
#else

      if (verbose >= 1) {
         llvm::errs() << "Constructing AEGPO for " << F.getName() << "\n";
      }

      AEGPO2 aegpo {F};
      if (!aegpo_output_path.empty()) {
         aegpo.dump_graph(format_graph_path(aegpo_output_path, F));
      }

      if (verbose >= 1) {
         llvm::errs() << "Constructing AEG for " << F.getName() << "\n";
      }


      ProfilerStart(format_graph_path("out/%s.prof", F).c_str());
      signal(SIGINT, [] (int sig) {
         ProfilerStop();
         std::exit(0);
      });
      AEG aeg {aegpo};
      aeg.construct(2, AA);
      // aeg.simplify();
      if (!aeg_output_path.empty()) {
         aeg.dump_graph(format_graph_path(aeg_output_path, F));
      }
      ProfilerStop();

      llvm::errs() << "Testing...\n";
      aeg.test();
      llvm::errs() << "done\n";

      // DEBUG: print out function, loop info
      auto& os = llvm::errs();
      os << "Loops:\n";
      aegpo.dump_loops(os);
      os << "Funcs:\n";
      aegpo.dump_funcs(os);

#endif
      
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



/* NOTES 
 * We only consider these instructions:
 * - Memory accesses: load, store, clflush, etc.
 * - Branch instructions
 * - Terminator instructions: go to bottom.
 * 
 * I could do this in multiple passes, in the LLVM spirit, to simplify work and make intermediate
 * results more verifiable.
 * 
 * First, we can squash out all irrelevant instructions and construct a basic control flow graph
 * from relevant instructions. We don't even need branches, either; we can model branching as edges
 * looping back.
 * However, we still need to model dataflow dependencies, like a very simple taint analysis.
 * We can use this analysis to extract ...
 *
 * Address Dependency Pass
 * This will find all address dependencies in the program.
 * These take the form of a value being used in pointer arithmetic on an address that may be 
 * accessed.
 *
 * For each variable, track the set of load instructions whose results were used to compute its 
 * value. Then, when you see a store, add addr dependencies for the corresponding set of the 
 * address.
 * 
 * I := set of instructions
 * Top := {}
 * Bot := I
 * Transition function f(i in I) := ...
 * Meet := union
 * Direction := forward
 */
