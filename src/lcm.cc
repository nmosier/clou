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
#include <llvm/Analysis/CallGraph.h>
#include <llvm/IR/IRBuilder.h>

#include "lcm.h"
#include "config.h"
#include "aeg/aeg.h"
#include "cfg/expanded.h"
#include "cfg/unrolled.h"
#include "cfg/calls.h"
#include "util/profiler.h"
#include "util/llvm.h"
#include "util/output.h"
#include "db.h"
#include "mon/proto.h"
#include "mon/client.h"
#include "cfg/block.h"
#include "util/exception.h"
#include "util/output.h"
#include "util/scope.h"
#include "util/timer.h"
#include "util/algorithm.h"

Timer timer {nullptr};
std::string get_time() {
    return timer.get_str();
}

template <typename Graph>
void output_(const Graph& graph, const std::string& name, const llvm::Function& F) {
    if (!output_dir.empty()) {
        std::stringstream ss;
        ss << output_dir << "/" << name << "-" << F.getName().str() << ".dot";
        graph.dump_graph(ss.str());
    }
}

struct LCMPass: public llvm::ModulePass {
    static char ID;
    
    LCMPass(): llvm::ModulePass(ID) {}
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& usage) const override {
        usage.addRequiredTransitive<llvm::AAResultsWrapperPass>();
        usage.addRequiredTransitive<llvm::CallGraphWrapperPass>();
    }
    
    virtual bool runOnModule(llvm::Module& M) override {
        ::signal(SIGSEGV, SIG_DFL);
        ::signal(SIGABRT, SIG_DFL);
        
        Transmitters transmitters;
        
        for (llvm::Function *F : getFunctionOrder(M)) {
            if (F->isDeclaration()) {
                std::cerr << "skipping function declaration " << F->getName().str() << "\n";
            } else {
                llvm::AliasAnalysis& AA = getAnalysis<llvm::AAResultsWrapperPass>(*F).getAAResults();
                open_log(F->getName().str());
                timer = Timer(nullptr); // reset timer
                runOnFunction(*F, AA, TransmitterOutputIt(transmitters, transmitters.end()));
                close_log();
            }
        }
        
        bool res = false;
        if (fence_insertion) {
            res = insert_fences(transmitters);
            
        }
        
        return res;
    }
    
    std::vector<llvm::Function *> getFunctionOrder(llvm::Module& M) {
        // want to find all roots in call graph
        llvm::CallGraph& CG = getAnalysis<llvm::CallGraphWrapperPass>().getCallGraph();

        std::vector<llvm::Function *> order;
        std::transform(M.begin(), M.end(), std::back_inserter(order), [] (llvm::Function& F) {
            return &F;
        });
        for (const llvm::Function *F : order) {
            std::cerr << F->getName().str() << ": " << CG[F]->getNumReferences() << "\n";
        }
        std::sort(order.begin(), order.end(), [&] (const llvm::Function *F1, const llvm::Function *F2) -> bool {
            return CG[F1]->getNumReferences() < CG[F2]->getNumReferences();
        });
        return order;
    }
    
    template <typename OutputIt>
    void runOnFunction(llvm::Function& F, llvm::AliasAnalysis& AA, OutputIt out) {
        const std::string func = F.getName().str();
        
        llvm::errs() << "processing function '" << F.getName() << "'\n";
        
        if (client) {
            mon::Message msg;
            msg.mutable_func_started()->mutable_func()->set_name(F.getName().str());
            client.send(msg);
        }
        
        
        const auto profiler_stop = util::defer([] () {
            if (profile) {
                ProfilerStop();
            }
        });
        if (profile) {
            std::stringstream ss;
            ss << output_dir << "/" << func << ".prof";
            ProfilerStart(ss.str().c_str());
            const auto handler = [] (int sig) {
                ProfilerStop();
                if (sig == SIGINT) {
                    std::exit(1);
                }
            };
            signal(SIGINT, handler);
            signal(SIGUSR1, handler);
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
                
                return;
            }
            
            if (!function_names.empty()) {
                bool match = false;
                for (const std::string& function_regex : function_names) {
                    if (std::regex_match(static_cast<std::string>(F.getName()), std::regex {function_regex})) {
                        match = true;
                        break;
                    }
                }
                if (!match) { return; }
            }
            
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
            cfg_calls.sort(); // EXPERIMENTAL
            
            if (output_cfgs.calls) {
                output_(cfg_calls, "cfg-calls", F);
            }
            std::cerr << "cfg-calls: " << cfg_calls.size() << " nodes\n";
            
            logv(1, "Constructing expanded AEGPO for " << F.getName() << "\n");
            client.send_step("cfg-expanded", F.getName().str());
            CFG_Expanded cfg_expanded {spec_depth};
            cfg_expanded.construct(cfg_calls);
            
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
            aeg::AEG aeg {cfg_expanded, AA};
            aeg.construct(AA, rob_size);
            
            // DEBUG INFO
            {
                unsigned i = 0;
                aeg.for_each_edge(aeg::Edge::ADDR_GEP, [&i] (NodeRef, NodeRef, const auto&) { ++i; });
                logv(1, "num ADDR_GEP edges: " << i << "\n");
            }
            
            

            client.send_step("leakage", F.getName().str());
            llvm::errs() << "Testing...\n";
            Transmitters transmitters;
            aeg.test(TransmitterOutputIt(transmitters, transmitters.end()));
            llvm::errs() << "done\n";
            
            // find the first transmitter in topological order
            for (NodeRef ref : cfg_expanded.reverse_postorder()) {
                const auto& node = cfg_expanded.lookup(ref);
                const auto *I = std::visit(util::overloaded {
                    [&] (const llvm::Instruction *I) -> const llvm::Instruction * {
                        return I;
                    },
                    [&] (const CFG::Node::Call& call) -> const llvm::Instruction *   {
                        return call.C;
                    },
                    [&] (Entry) -> const llvm::Instruction * { return nullptr; },
                    [&] (Exit) -> const llvm::Instruction * { return nullptr; },
                }, node.v);
                if (I != nullptr) {
                    if (transmitters.contains(I)) {
                        *out++ = I;
                        break;
                    }
                }
            }
            
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

                mon::Message msg;
                for (const std::string& s : set) {
                    analyzed_functions.insert(s);
                    msg.mutable_funcs_analyzed()->add_funcs()->set_name(s);
                }
                client.send(msg);
            }
            
        } catch (const util::resume& resume) {
            std::cerr << resume.what() << "\n";
        }
        
        if (client) {
            mon::Message msg;
            msg.mutable_func_completed()->mutable_func()->set_name(F.getName().str());
            client.send(msg);
        }
        
        std::cerr << "ANALYZED: " << cpu_time() << "\n";
        std::cerr << "done analyzing function " << F.getName().str() << "\n";
        
        return;
    }
    
    bool insert_fences(const Transmitters& transmitters) const {
        std::cerr << transmitters.size() << " transmitters\n";
        for (const llvm::Instruction *transmitter_ : transmitters) {
            llvm::Instruction *transmitter = const_cast<llvm::Instruction *>(transmitter_);
            llvm::IRBuilder<> builder(transmitter);
            builder.SetInsertPoint(transmitter);
            llvm::FenceInst *FI = builder.CreateFence(llvm::AtomicOrdering::Acquire);
            llvm::errs() << "INSERTED FENCE: " << *FI << "\n";
        }
        return !transmitters.empty();
    }
};

char LCMPass::ID = 0;

// Automatically enable the pass.
// http://adriansampson.net/blog/clangpass.html
namespace {
void registerLCMPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new LCMPass());
}
llvm::RegisterStandardPasses RegisterMyPass {
    llvm::PassManagerBuilder::EP_EnabledOnOptLevel0,
    registerLCMPass
};
llvm::RegisterStandardPasses RegisterMyPass0 {
    llvm::PassManagerBuilder::EP_ModuleOptimizerEarly,
    registerLCMPass
};

llvm::RegisterPass<LCMPass> X {
    "lcm", "LCM Pass"
};


}
