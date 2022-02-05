
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

#include <set>
#include <compare>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <string_view>

#include "attacker-taint.h"

constexpr unsigned stb_size = 10;

struct STBEntry {
    const llvm::Value *pointer;
    bool arch_controlled;
    unsigned staleness;
    
    auto operator<=>(const STBEntry&) const = default;
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const STBEntry& stb_ent) {
    os << stb_ent.staleness << " " << (stb_ent.arch_controlled ? "controlled" : "uncontrolled") << " " << *stb_ent.pointer;
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
            std::set<const llvm::Value *> commit_controlled, commit_uncontrolled;
            for (STBEntry stb_ent : in.stb) {
                if (++stb_ent.staleness >= stb_size) {
                    (stb_ent.arch_controlled ? commit_controlled : commit_uncontrolled).insert(stb_ent.pointer);
                } else {
                    out.stb.insert(stb_ent);
                }
            }
            
            // commit uncontrolled
            std::copy(commit_uncontrolled.begin(), commit_uncontrolled.end(), std::inserter(out.mem, out.mem.end()));
            
            // commit controlled
            for (const llvm::Value *controlled : commit_controlled) {
                std::erase_if(out.mem, [&] (const llvm::Value *uncontrolled) -> bool {
                    return AA.alias(controlled, uncontrolled) != llvm::NoAlias;
                });
            }
            
            // add stored value
            const llvm::Value *pointer = SI->getPointerOperand();
            STBEntry new_ent = {
                .pointer = pointer,
                .arch_controlled = attacker_taint.get(pointer),
                .staleness = 0,
            };
            out.stb.insert(new_ent);
            
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
        const auto copy_uncontrolled_stb = [&out] (const STB& stb) {
            std::copy_if(stb.begin(), stb.end(), std::inserter(out.stb, out.stb.end()), [] (const STBEntry& stb_ent) -> bool {
                return stb_ent.arch_controlled;
            });
        };
        copy_uncontrolled_stb(a.stb);
        copy_uncontrolled_stb(b.stb);
        
        for (const STBEntry& ent_a : a.stb) {
            if (ent_a.arch_controlled) { continue; }
            for (const STBEntry& ent_b : b.stb) {
                if (ent_b.arch_controlled) { continue; }
                if (AA.alias(ent_a.pointer, ent_b.pointer) == llvm::MustAlias) {
                    const unsigned staleness = std::min(ent_a.staleness, ent_b.staleness);
                    STBEntry ent_out1 = {
                        .pointer = ent_a.pointer,
                        .arch_controlled = false,
                        .staleness = staleness,
                    };
                    STBEntry ent_out2 = {
                        .pointer = ent_b.pointer,
                        .arch_controlled = false,
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
        
#if 1
        AttackerTaintResults attacker_taint = getAnalysis<AttackerTaintPass>().getResults();
#endif
        
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
        std::transform(stores.begin(), stores.end(), std::inserter(top.stb, top.stb.end()), [] (const llvm::Value *pointer) -> STBEntry {
            return {
                .pointer = pointer,
                .staleness = stb_size - 1,
                .arch_controlled = false,
            };
        });
        
        IMap i_ins, i_outs;
        LMap l_ins, l_outs;
        
        for_each_instruction(F, [&i_ins, &i_outs, &top] (const llvm::Instruction *I) {
            i_ins.insert_or_assign(I, top);
            i_outs.insert_or_assign(I, top);
        });
        
        const llvm::Instruction *entry = &F.getEntryBlock().front();
        
        i_ins.at(entry) = Value();

        // Dataflow Algorithm
        
#if 1
        bool changed;
        
        do {
            changed = false;
            
            for (const llvm::BasicBlock& B : F) {
                for (const llvm::Instruction& I : B) {
                    
                    llvm::errs() << "Instruction: " << I << "\n";
                    
                    /* meet operator */
                    if (&I != entry) {
                        // if block entry
                        if (&I == &B.front()) {
                            
                            auto& in = i_ins.at(&I);
                            Value new_in = top;
                            for (const llvm::BasicBlock *B_pred : llvm::predecessors(&B)) {
                                const llvm::Instruction *I = B_pred->getTerminator();
                                new_in = Value::meet(new_in, i_outs.at(I), AA);
                            }
                            if (in != new_in) { changed = true; }
                            in = new_in;
                            
                        } else {
                            
                            const llvm::Instruction *I_prev = I.getPrevNode();
                            const auto& out = i_outs.at(I_prev);
                            auto& in = i_ins.at(&I);
                            if (in != out) { changed = true; }
                            i_ins.at(&I) = i_outs.at(I_prev);
                            
                        }
                        
                    }
                    
                    llvm::errs() << "IN:\n" << i_ins.at(&I);
                    
                    /* transfer ins -> outs */
                    {
                        auto& out = i_outs.at(&I);
                        auto new_out = Value::transfer(i_ins.at(&I), &I, AA, attacker_taint);
                        if (out != new_out) { changed = true; }
                        out = new_out;
                    }
                    
                    llvm::errs() << "OUT:\n" << i_outs.at(&I);
                    
                    llvm::errs() << "\n";
                }
            }
            

            if (!changed) {
                // print out results
                
                llvm::errs() << "Updated Results:\n";
                
                for_each_instruction(F, [&] (const llvm::Instruction *I) {
                    llvm::errs() << "Instruction: " << *I << "\n";
                    
                    const Value& in = i_ins.at(I);
                    llvm::errs() << "Store Buffer:\n";
                    for (const STBEntry& stb_ent : in.stb) {
                        llvm::errs() << stb_ent.staleness << " ";
                        llvm::errs() << (stb_ent.arch_controlled ? "controlled" : "uncontrolled") << " " << *stb_ent.pointer << "\n";
                    }
                    
                    llvm::errs() << "Memory:\n";
                    for (const llvm::Value *pointer : in.mem) {
                        llvm::errs() << *pointer << "\n";
                    }
                    
                });
                
                llvm::errs() << "\n\n\n\n\n\n\n";
            }
            
        } while (changed);
        
        // Pretty-print results:
        // For each load, print whether it can speculatively load controlled data.
        llvm::errs() << "Results:\n";
        llvm::errs() << "Num stores = " << stores.size() << "\n";
        for_each_instruction(F, [&] (const llvm::Instruction *I) {
            if (const llvm::LoadInst *LI = llvm::dyn_cast<llvm::LoadInst>(I)) {
                const llvm::Value *pointer = LI->getPointerOperand();
                const auto& in = i_ins.at(LI);
                const auto stb_it = std::find_if(in.stb.begin(), in.stb.end(), [&] (const STBEntry& stb_entry) -> bool {
                    return AA.alias(pointer, stb_entry.pointer) != llvm::NoAlias && stb_entry.arch_controlled;
                });
                const auto mem_it = std::find_if(in.mem.begin(), in.mem.end(), [&] (const llvm::Value *mem_pointer) -> bool {
                    return AA.alias(pointer, mem_pointer) == llvm::MustAlias;
                });
                
                const bool controlled = (stb_it != in.stb.end() || mem_it == in.mem.end());
                llvm::errs() << *LI << " - ";
                if (controlled) {
                    llvm::errs() << "controlled";
                } else {
                    llvm::errs() << "uncontrolled";
                }
                llvm::errs() << "\n";
            }
        });

#endif
        
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

