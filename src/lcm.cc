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
#include "config.h"
#include "cfg.h"
#include "aeg.h"
#include "aeg-po2.h"
#include "aeg-expanded.h"

using llvm::errs;

template <typename Graph>
void output(const Graph& graph, const std::string& name, const llvm::Function& F) {
   if (!output_dir.empty()) {
      graph.dump_graph(format_graph_path(output_dir + "/" + name + "-%s.dot", F));
   }
}
   
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
      logv(1) << "Constructing AEGPO for " << F.getName() << "\n";
      AEGPO2 aegpo {F};
      output(aegpo, "aegpo", F);

      logv(1) << "Constructing expanded AEGPO for " << F.getName() << "\n";
      AEGPO_Expanded aegpo_expanded {aegpo};
      logv(2) << "Expanded AEGPO node counts: " << aegpo.size() << " (orig) vs. "
              << aegpo_expanded.size() << " (expanded)\n";
      output(aegpo_expanded, "aegpoexp", F);
      
      logv(1) << "Constructing AEG for " << F.getName() << "\n";
      ProfilerStart(format_graph_path("out/%s.prof", F).c_str());
      signal(SIGINT, [] (int sig) {
         ProfilerStop();
         std::exit(0);
      });
      AEG aeg {aegpo_expanded};
      aeg.construct(2, AA);
      output(aeg, "aeg", F);
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
