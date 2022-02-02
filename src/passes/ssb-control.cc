
#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/AliasAnalysis.h>

#include <vector>

#include "attacker-taint.h"

constexpr unsigned stb_size = 5;

struct STBEntry {
    const llvm::Value *pointer;
    bool arch_controlled;
    unsigned staleness;
};

struct Value {
    using STB = std::vector<STBEntry>;
    using Memory = std::vector<const llvm::Value *>;
    
    STB stb; /*!< Store buffer contents. */
    Memory mem; /*!< Uncontrolled memory, represented as collection of uncontrolled pointers. */
    
    void graduate(llvm::AliasAnalysis& AA) {
        for (auto stb_it = stb.begin(); stb_it != stb.end(); ) {
            if (++stb_it->staleness >= stb_size) {
                
                // move to memory
                commit_to_memory(*stb_it, AA);
    
                // erase from store buffer
                stb_it = stb.erase(stb_it);
                
            } else {
                ++stb_it;
            }
        }
    }
    
    void commit_to_memory(const STBEntry& ent, llvm::AliasAnalysis& AA) {
        if (ent.arch_controlled) {
            // arch-controlled, so remove all maybe-aliasing arch-uncontrolled pointers in memory
            std::erase_if(mem, [&] (const llvm::Value *store) {
                return AA.alias(store, ent.pointer) != llvm::NoAlias;
            });
        } else {
            // arch-uncontrolled, so add to set
            mem.push_back(ent.pointer);
        }
    }
    
    void transfer(const llvm::Instruction *I, llvm::AliasAnalysis& AA, const AttackerTaintResults& attacker_taint) {
        if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
            
            graduate(AA);
            
            const llvm::Value *pointer = SI->getPointerOperand();
            const STBEntry stb_ent = {
                .pointer = pointer,
                .arch_controlled = attacker_taint.get(pointer),
                .staleness = 0,
            };
            stb.push_back(stb_ent);
            
        } else if (const llvm::CallBase *CB = llvm::dyn_cast<llvm::CallBase>(I)) {
            
            // assume that store buffer is flushed and all uncontrolled memory overwritten
            stb.clear();
            mem.clear();
            
        }
    }
    

    static Value meet(const Value& a, const Value& b, llvm::AliasAnalysis& AA) {
        Value out;
        
        // Meet STB
        const auto copy_uncontrolled_stb = [&out] (const STB& stb) {
            std::copy_if(stb.begin(), stb.end(), std::back_inserter(out.stb), [] (const STBEntry& stb_ent) -> bool {
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
                    out.stb.push_back(ent_out1);
                    out.stb.push_back(ent_out2);
                }
            }
        }
        
        // Meet memory (alias intersection)
        for (const llvm::Value *store_a : a.mem) {
            for (const llvm::Value *store_b : b.mem) {
                if (AA.alias(store_a, store_b) == llvm::MustAlias) {
                    out.mem.push_back(store_a);
                    out.mem.push_back(store_b);
                }
            }
        }
        
        return out;
    }
};

struct SSBControlPass final: public llvm::FunctionPass {
    static inline char ID = 0;
    
    SSBControlPass(): llvm::FunctionPass(ID) {}
    
    virtual void getAnalysisUsage(llvm::AnalysisUsage& AU) const override {
        AU.addRequired<AttackerTaintPass>();
        AU.setPreservesAll();
    }
    
    virtual bool runOnFunction(llvm::Function& F) override {
        
    }
};
