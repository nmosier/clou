
#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <llvm/IR/IntrinsicInst.h>
#include <llvm/Analysis/LoopInfo.h>
#include <llvm/IR/Operator.h>

#include <gperftools/profiler.h>

#include <set>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <string_view>
#include <tuple>

#include "attacker-taint.h"
#include "dataflow/dataflow.h"

constexpr unsigned stb_size = 1;

struct STBEntry {
    const llvm::Value *pointer;
    unsigned staleness;
    
    bool operator==(const STBEntry&) const = default;
    
    auto tuple() const {
        return std::make_tuple(pointer, staleness);
    }

    bool operator<(const STBEntry& e) const {
        return tuple() < e.tuple();
    }
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const STBEntry& stb_ent) {
    os << stb_ent.staleness << " uncontrolled " << *stb_ent.pointer;
    return os;
}

struct Value {
    using STB = std::set<STBEntry>;
    using Memory = std::set<const llvm::Value *>;
    
    STB stb; /*!< Store buffer contents. */
    Memory mem; /*!< Uncontrolled memory, represented as collection of uncontrolled pointers. */
    
    bool operator==(const Value&) const = default;
    
    static Value transfer(const Value& in, const llvm::Instruction *I, llvm::AliasAnalysis& AA, const AttackerTaintResults& attacker_taint) {
        
        if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
            
            Value out;
            
            // transfer mem
            out.mem = in.mem;
            
            // transfer stb
            std::set<const llvm::Value *> commit;
            for (STBEntry stb_ent : in.stb) {
                if (++stb_ent.staleness >= stb_size) {
                    // commit to memory
                    out.mem.insert(stb_ent.pointer);
                } else {
                    // update in STB
                    out.stb.insert(stb_ent);
                }
            }
            
            // add stored value
            const llvm::Value *pointer = SI->getPointerOperand();
            
            if (attacker_taint.get(pointer)) {
                
                // remove all aliasing entries in STB and MEM
                const auto invalidate = [&AA, pointer] (const llvm::Value *V) {
                    return AA.alias(V, pointer) != llvm::NoAlias;
                };
                
                std::erase_if(out.stb, [&] (const STBEntry& stb_ent) -> bool {
                    return invalidate(stb_ent.pointer);
                });
                std::erase_if(out.mem, invalidate);
                
            } else {
                
                STBEntry new_ent = {
                    .pointer = pointer,
                    .staleness = 0,
                };
                out.stb.insert(new_ent);
                
            }
            
            return out;
            
        } else if (llvm::isa<llvm::DbgInfoIntrinsic>(I)) {

            return in;
            
        } else if (llvm::isa<llvm::CallBase>(I)) {
            
            return Value();
            
        } else {
            
            return in;
            
        }
    }

    static Value meet(const Value& a, const Value& b, llvm::AliasAnalysis& AA) {
        Value out;
        
        // Meet STB
        const auto copy_stb = [&out] (const STB& stb) {
            std::copy(stb.begin(), stb.end(), std::inserter(out.stb, out.stb.end()));
        };
        copy_stb(a.stb);
        copy_stb(b.stb);
        
        for (const STBEntry& ent_a : a.stb) {
            for (const STBEntry& ent_b : b.stb) {
                if (AA.alias(ent_a.pointer, ent_b.pointer) == llvm::MustAlias) {
                    const unsigned staleness = std::min(ent_a.staleness, ent_b.staleness);
                    STBEntry ent_out1 = {
                        .pointer = ent_a.pointer,
                        .staleness = staleness,
                    };
                    STBEntry ent_out2 = {
                        .pointer = ent_b.pointer,
                        .staleness = staleness,
                    };
                    out.stb.insert(ent_out1);
                    out.stb.insert(ent_out2);
                }
            }
        }
        
        // Meet memory (alias intersection)
        for (const llvm::Value *store_a : a.mem) {
            for (const llvm::Value *store_b : b.mem) {
                if (AA.alias(store_a, store_b) == llvm::MustAlias) {
                    out.mem.insert(store_a);
                    out.mem.insert(store_b);
                }
            }
        }
        
        return out;
    }
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const Value& value) {
    os << "Store Buffer:\n";
    for (const STBEntry& stb_ent : value.stb) {
        os << stb_ent << "\n";
    }
    os << "Memory:\n";
    for (const llvm::Value *pointer : value.mem) {
        os << *pointer << "\n";
    }
    return os;
}

struct SSBControlPass final: public llvm::FunctionPass {
    static inline char ID = 0;
    
    SSBControlPass(): llvm::FunctionPass(ID) {}
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override {
#if 1
        AU.addRequired<AttackerTaintPass>();
#endif
        AU.addRequired<llvm::AAResultsWrapperPass>();
        AU.addRequired<llvm::LoopInfoWrapperPass>();
        AU.setPreservesAll();
    }
    
    template <class Func>
    static void for_each_instruction(const llvm::Function& F, Func func) {
        for (const llvm::BasicBlock& B : F) {
            for (const llvm::Instruction& I : B) {
                func(&I);
            }
        }
    }
    
    using IMap = std::unordered_map<const llvm::Instruction *, Value>;
    using LMap = std::unordered_map<const llvm::Loop *, Value>;
    
    static unsigned get_min_loop_iterations(const llvm::Loop *L) {
        std::unordered_set<const llvm::BasicBlock *> blocks;
        std::copy(L->block_begin(), L->block_end(), std::inserter(blocks, blocks.end()));
        for (const llvm::Loop *subloop : *L) {
            for (const llvm::BasicBlock *subblock : subloop->blocks()) {
                blocks.erase(subblock);
            }
        }
        
        for (const llvm::BasicBlock *B : blocks) {
            for (const llvm::Instruction& I : *B) {
                if (const llvm::IntrinsicInst *II = llvm::dyn_cast<llvm::IntrinsicInst>(&I)) {
                    if (II->getIntrinsicID() == llvm::Intrinsic::var_annotation) {
                        const llvm::Value *V = llvm::cast<llvm::GEPOperator>(II->getArgOperand(1));
                        const llvm::GlobalVariable *GV = llvm::cast<llvm::GlobalVariable>(V);
                        const llvm::ConstantDataArray *CDA = llvm::cast<llvm::ConstantDataArray>(GV->getInitializer());
                        const std::string s = CDA->getAsCString().str();
                        
                        std::string_view sv = s;
                        const auto pos = sv.find('=');
                        if (pos != std::string_view::npos) {
                            std::string_view key = sv.substr(0, pos);
                            std::string_view value = sv.substr(pos + 1);
                            if (key == "loop.min") {
                                return std::stoul(std::string(value));
                            }
                        }
                    }
                }
            }
        }
        
        return 0;
    }
    
    virtual bool runOnFunction(llvm::Function& F) override {
        llvm::errs() << "\n\nFunction " << F.getName() << "\n\n";
        
        llvm::AliasAnalysis& AA = getAnalysis<llvm::AAResultsWrapperPass>().getAAResults();
        AttackerTaintResults attacker_taint = getAnalysis<AttackerTaintPass>().getResults();
        llvm::LoopInfo& LI = getAnalysis<llvm::LoopInfoWrapperPass>().getLoopInfo();
        
        std::set<const llvm::Value *> stores;
        for_each_instruction(F, [&stores] (const llvm::Instruction *I) {
            if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
                stores.insert(SI->getPointerOperand());
            }
        });
        
        for (auto it1 = stores.begin(); it1 != stores.end(); ++it1) {
            for (auto it2 = it1; it2 != stores.end(); ++it2) {
                llvm::errs() << "AA: " << **it1 << " - " << **it2 << " - ";
                llvm::errs() << AA.alias(*it1, *it2);
                llvm::errs() << "\n";
            }
        }
        
        Value top = {
            .stb = Value::STB(),
            .mem = stores,
        };
#if 0
        std::transform(stores.begin(), stores.end(), std::inserter(top.stb, top.stb.end()), [] (const llvm::Value *pointer) -> STBEntry {
            return {
                .pointer = pointer,
                .staleness = stb_size - 1,
                .arch_controlled = false,
            };
        });
#else
        for (const llvm::Value *pointer : stores) {
            for (unsigned i = 0; i < stb_size; ++i) {
                top.stb.insert({
                    .pointer = pointer,
                    .staleness = i,
                });
            }
        }
#endif
        
        IMap i_ins, i_outs;
        
        // Dataflow Algorithm
        
        using Dataflow = dataflow::Dataflow<Value>;
        Dataflow::Context context = {
            .top = top,
            .transfer = [&AA, &attacker_taint] (const llvm::Instruction *I, const Value& in) -> Value {
                return Value::transfer(in, I, AA, attacker_taint);
            },
                .meet = [&AA] (const Value& a, const Value& b) -> Value {
                    return Value::meet(a, b, AA);
                },
        };
        
        Dataflow::Map exit_values;
        Dataflow::Function function {context, &F, &LI, Dataflow::Function::Mode::LOOP};
        function.transfer(Value(), i_ins, i_outs, exit_values);
        
        // Pretty-print results:
        // For each load, print whether it can speculatively load controlled data.
        llvm::errs() << "Results:\n";
        llvm::errs() << "Num stores = " << stores.size() << "\n";
        for_each_instruction(F, [&] (const llvm::Instruction *I) {
            if (const llvm::LoadInst *LI = llvm::dyn_cast<llvm::LoadInst>(I)) {
                const llvm::Value *pointer = LI->getPointerOperand();
                const auto& in = i_ins.at(LI);
                const auto mem_it = std::find_if(in.mem.begin(), in.mem.end(), [&] (const llvm::Value *mem_pointer) -> bool {
                    return AA.alias(pointer, mem_pointer) == llvm::MustAlias;
                });
                const bool controlled = (mem_it == in.mem.end());
                llvm::errs() << *LI << " - ";
                if (controlled) {
                    llvm::errs() << "controlled";
                } else {
                    llvm::errs() << "uncontrolled";
                }
                llvm::errs() << "\n";
            }
        });

        llvm::errs() << "\n\n\n Arch Control Results:\n" << attacker_taint << "\n";
        
        llvm::errs() << "Function " << F.getName() << " done\n";
        
        return false;
    }
};



namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new SSBControlPass());
}

llvm::RegisterStandardPasses registerCFP0 {
    llvm::PassManagerBuilder::EP_EarlyAsPossible,
    registerPass,
};

llvm::RegisterPass<SSBControlPass> X {
    "ssb_control", "SSB Control Pass",
};

}



namespace {

#if 0
struct Profiler {
    Profiler(const std::string& name) {
        ProfilerStart(name.c_str());
    }
    
    ~Profiler() {
        ProfilerStop();
    }
};

Profiler profiler {"prof"};
#endif

}
