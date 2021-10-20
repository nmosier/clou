#include <optional>
#include <string>
#include <signal.h>
#include <regex>

#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/AliasAnalysis.h>

#include "util.h"
#include "lcm.h"
#include "config.h"
#include "aeg/aeg.h"
#include "cfg/expanded.h"
#include "cfg/unrolled.h"
#include "cfg/calls.h"
#include "profiler.h"
#include "util/llvm.h"
#include "util/output.h"

using llvm::errs;

template <typename Graph>
void output_(const Graph& graph, const std::string& name, const llvm::Function& F) {
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
        try {
            if (!function_names.empty()) {
                bool match = false;
                for (const std::string& function_regex : function_names) {
                    if (std::regex_match(static_cast<std::string>(F.getName()), std::regex {function_regex})) {
                        match = true;
                        break;
                    }
                }
                if (!match) { return false; }
            }
            
            llvm::AliasAnalysis& AA = getAnalysis<llvm::AAResultsWrapperPass>().getAAResults();
            
            const unsigned num_unrolls = 2;
            
            /* Construct AEG */
            logv(1) << "Constructing AEGPO for " << F.getName() << "\n";
            CFG_Unrolled aegpo_unrolled {F, spec_depth, num_unrolls};
            aegpo_unrolled.construct();
            
            CFG_Calls cfg_calls {spec_depth};
            cfg_calls.construct(aegpo_unrolled);
            output_(cfg_calls, "calls", F);
            
            std::cerr << "outputting\n";
            output_(aegpo_unrolled, "aegpo", F);
            
            logv(1) << "Constructing expanded AEGPO for " << F.getName() << "\n";
            CFG_Expanded aegpo_expanded {spec_depth};
            {
                switch (leakage_class) {
                    case LeakageClass::SPECTRE_V1: {
                        Expand_SpectreV1 expand_spectre_v1 {cfg_calls, spec_depth};
                        aegpo_expanded.construct(cfg_calls, expand_spectre_v1);
                        break;
                    }
                        
                    case LeakageClass::SPECTRE_V4: {
                        Expand_SpectreV4 expand_spectre_v4 {cfg_calls, spec_depth};
                        aegpo_expanded.construct(cfg_calls, expand_spectre_v4);
                        break;
                    }
                        
                    default: std::abort();
                }
            }
            logv(2) << "Expanded AEGPO node counts: " << aegpo_unrolled.size() << " (orig) vs. "
            << aegpo_expanded.size() << " (expanded)\n";
            output_(aegpo_expanded, "aegpoexp", F);
            
#if 1
            // DEBUG: show refs
            for (NodeRef ref = 0; ref < aegpo_expanded.size(); ++ref) {
                using namespace output;
                const auto& node = aegpo_expanded.lookup(ref);
                llvm::errs() << ref << ": " << node << ": {\n";
                for (const auto& pair : node.refs) {
                    llvm::errs() << "  " << *pair.first << ": " << pair.second << ",\n";
                }
                std::cerr << "}\n";
            }
#endif
            
            
            logv(1) << "Constructing AEG for " << F.getName() << "\n";
            ProfilerStart(format_graph_path("out/%s.prof", F).c_str());
            signal(SIGINT, [] (int sig) {
                ProfilerStop();
                std::exit(0);
            });
            aeg::AEG aeg {aegpo_expanded};
            aeg.construct(AA, rob_size);
#if 0
            output(aeg, "aeg", F);
#endif
            ProfilerStop();
            
            llvm::errs() << "Testing...\n";
            aeg.test();
            llvm::errs() << "done\n";
            
        } catch (const util::resume& resume) {
            std::cerr << resume.what() << "\n";
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
