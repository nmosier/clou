#include <optional>
#include <string>
#include <signal.h>
#include <regex>
#include <unistd.h>

#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/AliasAnalysis.h>

#include "lcm.h"
#include "config.h"
#include "aeg/aeg.h"
#include "cfg/expanded.h"
#include "cfg/unrolled.h"
#include "cfg/calls.h"
#include "profiler.h"
#include "util/llvm.h"
#include "util/output.h"
#include "db.h"
#include "mon/proto.h"
#include "mon/client.h"
#include "cfg/block.h"
#include "util/exception.h"

using llvm::errs;

template <typename Graph>
void output_(const Graph& graph, const std::string& name, const llvm::Function& F) {
    if (!output_dir.empty()) {
        graph.dump_graph(format_graph_path(output_dir + "/" + name + "-%s.dot", F));
    }
}

struct LCMPass: public llvm::FunctionPass {
    static char ID;
    
    LCMPass(): FunctionPass(ID) {}
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& usage) const override {
        usage.addRequired<llvm::AAResultsWrapperPass>();
    }
    
    virtual bool runOnFunction(llvm::Function& F) override {
        llvm::errs() << "processing function '" << F.getName() << "'\n";
        
        if (client) {
            mon::Message msg;
            msg.mutable_func_started()->mutable_func()->set_name(F.getName().str());
            client.send(msg);
        }
        
        
        try {
            check_config();
            
            if (analyzed_functions.contains(F.getName().str())) {
                std::cerr << "skipping analyzed function " << F.getName().str() << "\n";
                if (client) {
                    mon::Message msg;
                    msg.mutable_func_completed()->mutable_func()->set_name(F.getName().str());
                    client.send(msg);
                }
                
                return false;
            }
            
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
            logv(1, "Constructing AEGPO for " << F.getName() << "\n");
            client.send_step("cfg-unrolled", F.getName().str());
            CFG_Unrolled aegpo_unrolled {F, spec_depth, num_unrolls};
            aegpo_unrolled.construct();
            
            if (output_cfgs.unrolled) {
                output_(aegpo_unrolled, "cfg-unrolled", F);
            }
            std::cerr << "cfg-unrolled: " << aegpo_unrolled.size() << " nodes\n";
            
            client.send_step("cfg-calls", F.getName().str());
            CFG_Calls cfg_calls {spec_depth};
            cfg_calls.construct(aegpo_unrolled);
            
            if (output_cfgs.calls) {
                output_(cfg_calls, "cfg-calls", F);
            }
            std::cerr << "cfg-calls: " << cfg_calls.size() << " nodes\n";
            
            logv(1, "Constructing expanded AEGPO for " << F.getName() << "\n");
            client.send_step("cfg-expanded", F.getName().str());
            CFG_Expanded cfg_expanded {spec_depth};
            {
                switch (leakage_class) {
                    case LeakageClass::SPECTRE_V1: {
                        Expand_SpectreV1 expand_spectre_v1 {cfg_calls, spec_depth};
                        cfg_expanded.construct(cfg_calls, expand_spectre_v1);
                        break;
                    }
                        
                    case LeakageClass::SPECTRE_V4: {
                        Expand_SpectreV4 expand_spectre_v4 {cfg_calls, spec_depth};
                        cfg_expanded.construct(cfg_calls, expand_spectre_v4);
                        break;
                    }
                        
                    default: std::abort();
                }
            }
            
            if (output_cfgs.expanded) {
                output_(cfg_expanded, "cfg-expanded", F);
            }
            std::cerr << "cfg-expanded: " << cfg_expanded.size() << " nodes\n";

            
            // DEBUG: show block CFG information
            {
                const BlockCFG bcfg {cfg_expanded};

                std::size_t block_size = 0.;
                for (const auto& p : bcfg.blocks) {
                    block_size += p.second.size();
                }
                std::cerr << "average block size in cfg-expanded: " << block_size / bcfg.blocks.size() << "\n";
            }
            
            client.send_property(F.getName().str(), "nodes", cfg_expanded.size());
            
            logv(1, "Constructing AEG for " << F.getName() << "\n");
            client.send_step("aeg", F.getName().str());
            aeg::AEG aeg {cfg_expanded};
            aeg.construct(AA, rob_size);
#if 0
            output(aeg, "aeg", F);
#endif
            
            ProfilerStart(format_graph_path("out/%s.prof", F).c_str());
            signal(SIGINT, [] (int sig) {
                ProfilerStop();
                std::exit(0);
            });

            client.send_step("leakage", F.getName().str());
            llvm::errs() << "Testing...\n";
            aeg.test();
            llvm::errs() << "done\n";
            
            ProfilerStop();
            
            // add analyzed functions
            {
                std::unordered_set<std::string> set;
                std::transform(aegpo_unrolled.nodes.begin(), aegpo_unrolled.nodes.end(), std::inserter(set, set.end()), [] (const CFG::Node& node) -> std::string {
                    if (const auto *Ip = std::get_if<const llvm::Instruction *>(&node.v)) {
                        return (**Ip).getFunction()->getName().str();
                    } else {
                        return "";
                    }
                });
                set.erase("");

                for (const std::string& s : set) {
                    analyzed_functions.insert(s);
                }
                
                {
                    mon::Message msg;
                    for (const std::string& s : set) {
                        msg.mutable_funcs_analyzed()->add_funcs()->set_name(s);
                    }
                    client.send(msg);
                }
            }
            
        } catch (const util::resume& resume) {
            std::cerr << resume.what() << "\n";
        }
        
        if (client) {
            mon::Message msg;
            msg.mutable_func_completed()->mutable_func()->set_name(F.getName().str());
            client.send(msg);
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
