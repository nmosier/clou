#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/MemoryDependenceAnalysis.h>
#include <llvm/IR/CFG.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <llvm/Support/raw_os_ostream.h>
#include <llvm/IR/IntrinsicInst.h>

#include <set>
#include <unordered_map>
#include <fstream>

#include "attacker-taint.h"
#include "config.h"

struct Value {
    /** Attacker-controlled instructions. */
    std::set<const llvm::Instruction *> insts;
    
    /** Non-attacker-controlled stores. Nullopt indicates universal set. */
    std::optional<std::set<const llvm::Value *>> stores;
    
    bool operator==(const Value& other) const = default;
};
using Map = std::unordered_map<const llvm::Instruction *, Value>;

Value meet(const Value& a, const Value& b, llvm::AliasAnalysis& AA) {
    Value out;
    std::set_union(a.insts.begin(), a.insts.end(), b.insts.begin(), b.insts.end(), std::inserter(out.insts, out.insts.end()));
    
    if (!a.stores) {
        out.stores = b.stores;
    } else if (!b.stores) {
        out.stores = a.stores;
    } else {
        
        out.stores.emplace();
        for (const llvm::Value *VA : *a.stores) {
            const auto VB_it = std::find_if(b.stores->begin(), b.stores->end(), [&AA, VA] (const llvm::Value *VB) -> bool {
                return AA.alias(VA, VB) == llvm::AliasResult::MustAlias;
            });
            if (VB_it != b.stores->end()) {
                out.stores->insert(VA);
                out.stores->insert(*VB_it);
            }
        }
        
    }

    return out;
}

bool in_taint(const llvm::Value *V, const Value& in) {
    if (const llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(V)) {
        return in.insts.contains(I);
    } else if (llvm::isa<llvm::Argument>(V)) {
        return true;
    } else if (llvm::isa<llvm::Constant, llvm::BasicBlock, llvm::MetadataAsValue>(V)) {
        return false;
    } else {
        llvm::errs() << "unhandled value " << *V << "\n";
        std::abort();
    }
}

Value transfer(const llvm::Instruction *I, const Value& in, llvm::AliasAnalysis& AA) {
    Value out = in;
    
    if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
        // check if operand is attacker-controlled
        const llvm::Value *V = SI->getValueOperand();
        const llvm::Value *P = SI->getPointerOperand();
        
        const bool taint = in_taint(V, in);
        if (taint) {
            // erase 'may alias' stores
            if (out.stores) {
                std::erase_if(*out.stores, [P, &AA] (const llvm::Value *V) -> bool {
                    return AA.alias(P, V) != llvm::NoAlias;
                });
            } else {
                out.stores.emplace();
            }
        } else {
            // erase 'must alias' stores
            if (out.stores) {
                std::erase_if(*out.stores, [P, &AA] (const llvm::Value *V) -> bool {
                    return AA.alias(P, V) == llvm::MustAlias;
                });
            } else {
                out.stores.emplace();
            }

            
            // add new store
            out.stores->insert(P);
        }
    } else if (const llvm::LoadInst *LI = llvm::dyn_cast<llvm::LoadInst>(I)) {
        // check if loaded value is ok
        // should find must-alias store
        
        const llvm::Value *L = LI->getPointerOperand();
        
        bool taint = true;
        for (const llvm::Value *S : in.stores.value_or(std::set<const llvm::Value *>())) {
            if (AA.alias(L, S) == llvm::MustAlias) {
                taint = false;
                break;
            }
        }
        
        if (taint) {
            out.insts.insert(LI);
        }
        
    } else if (llvm::isa<llvm::DbgInfoIntrinsic>(I)) {
        
        // do nothing
        
    } else if (const llvm::CallBase *CI = llvm::dyn_cast<llvm::CallBase>(I)) {
        
        // if the call writes to memory, mark everything as controlled
        if (!CI->onlyReadsMemory()) {
            out.stores = std::set<const llvm::Value *>();
        }
        
        // always mark call as attacker-tainted
        out.insts.insert(CI);
        
    } else {
        
        // taint is OR of input operands
        bool taint = false;
        for (const llvm::Value *V : I->operands()) {
            taint = taint || in_taint(V, in);
        }
        if (taint) {
            out.insts.insert(I);
        }
        
    }
    
    return out;
}

void AttackerTaintPass::getAnalysisUsage(llvm::AnalysisUsage& AU) const {
    AU.addRequired<llvm::AAResultsWrapperPass>();
    AU.setPreservesAll();
}

bool AttackerTaintPass::runOnFunction(llvm::Function& F) {
    results = AttackerTaintResults();
    results.F = &F;
    
    llvm::AliasAnalysis AA = std::move(getAnalysis<llvm::AAResultsWrapperPass>().getAAResults());
    
    bool changed;
    Map ins, outs;
    ins[&F.getEntryBlock().front()] = {.stores = std::set<const llvm::Value *>()};
    
    std::ofstream ofs;
    llvm::raw_os_ostream os {ofs};
    {
        std::stringstream ss;
        ss << output_dir << "/logs/" << F.getName().str() << ".controlled.log";
        ofs.open(ss.str());
    }
    
    const auto print = [&] () {
        // print results
        os << F << "\n";
        for (const llvm::BasicBlock& B : F) {
            for (const llvm::Instruction& I : B) {
                const auto desc = [&I] (const Map& map) {
                    if (map.at(&I).insts.contains(&I)) {
                        return "high";
                    } else {
                        return "low";
                    }
                };
                
                os << I << "\n";
                os << "store: ";
                const auto& out = outs.at(&I);
                if (out.stores) {
                    for (const llvm::Value *V : *out.stores) {
                        os << *V << "; ";
                    }
                } else {
                    os << "(universal)";
                }
                os << "\n";
                os << "OUT: " << desc(outs) << "\n";
            }
        }
        os << "\n\n\n";
    };
    
    do {
        changed = false;
        
        for (const llvm::BasicBlock& B : F) {
            for (const llvm::Instruction& I : B) {
                
                /* meet operator */
                {
                    // if block entry
                    if (&I == &B.front()) {
                        
                        auto& in = ins[&I];
                        Value new_in;
                        for (const llvm::BasicBlock *B_pred : llvm::predecessors(&B)) {
                            const llvm::Instruction *I = B_pred->getTerminator();
                            new_in = meet(new_in, outs[I], AA);
                        }
                        if (in != new_in) { changed = true; }
                        in = new_in;
                        
                    } else {
                        
                        const llvm::Instruction *I_prev = I.getPrevNode();
                        const auto& out = outs[I_prev];
                        auto& in = ins[&I];
                        if (in != out) { changed = true; }
                        ins[&I] = outs[I_prev];
                        
                    }
                    
                }
                
                /* transfer ins -> outs */
                {
                    auto& out = outs[&I];
                    auto new_out = transfer(&I, ins[&I], AA);
                    if (out != new_out) { changed = true; }
                    out = new_out;
                }
            }
        }
        
    } while (changed);

    print();
    
    // initialize results
    for (const llvm::BasicBlock& B : F) {
        for (const llvm::Instruction& I : B) {
            if (outs.at(&I).insts.contains(&I)) {
                results.insts.insert(&I);
            }
        }
    }
    
    return false;
}

namespace {

void registerPass(const llvm::PassManagerBuilder&, llvm::legacy::PassManagerBase& PM) {
    PM.add(new AttackerTaintPass());
}

llvm::RegisterStandardPasses registerCFP0 {
    llvm::PassManagerBuilder::EP_EarlyAsPossible,
    registerPass,
};

llvm::RegisterPass<AttackerTaintPass> X {
    "attacker_taint", "Attacker Taint Pass",
};

}

