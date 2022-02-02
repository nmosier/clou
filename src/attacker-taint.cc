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
#if CLOU
# include "config.h"
#endif

struct Value {
    using Insts = std::set<const llvm::Instruction *>;
    using Stores = std::set<const llvm::Value *>;
    
    /** Attacker-controlled instructions. */
    Insts insts;

    /** Non-attacker-controlled stores. Nullopt indicates universal set. */
    Stores stores;
    
    bool operator==(const Value& other) const = default;
    
    Value() = delete;
    Value(const Insts& insts, const Stores& stores): insts(insts), stores(stores) {}
};
using Map = std::unordered_map<const llvm::Instruction *, Value>;

Value meet(const Value& a, const Value& b, llvm::AliasAnalysis& AA) {
    Value out {Value::Insts(), Value::Stores()};
    
    // union controlled instructions
    std::set_union(a.insts.begin(), a.insts.end(), b.insts.begin(), b.insts.end(), std::inserter(out.insts, out.insts.end()));
    
    // only include pairs that must alias
    
    for (const llvm::Value *VA : a.stores) {
        const auto VB_it = std::find_if(b.stores.begin(), b.stores.end(), [&AA, VA] (const llvm::Value *VB) -> bool {
            return AA.alias(VA, VB) == llvm::AliasResult::MustAlias;
        });
        if (VB_it != b.stores.end()) {
            out.stores.insert(VA);
            out.stores.insert(*VB_it);
        }
    }

    return out;
}

bool in_taint(const llvm::Value *V, const Value& in) {
    if (const llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(V)) {
        return in.insts.contains(I);
    } else if (llvm::isa<llvm::Argument>(V)) {
        // If pointer, assume that it's not controlled.
        if (V->getType()->isPointerTy()) {
            return false;
        } else {
            return true;
        }
    } else if (llvm::isa<llvm::Constant, llvm::BasicBlock, llvm::MetadataAsValue>(V)) {
        return false;
    } else {
        llvm::errs() << "unhandled value " << *V << "\n";
        std::abort();
    }
}

llvm::AliasResult alias(const llvm::Value *P, const llvm::Value *V, llvm::AliasAnalysis& AA) {
    llvm::AliasResult alias_result = AA.alias(P, V);
    switch (alias_result) {
        case llvm::NoAlias:
        case llvm::MustAlias:
            return alias_result;
            
        case llvm::MayAlias: {
            
            // This is nasty. Should simplify this later.
            
            const auto is_integral = [] (const llvm::Value *V) -> bool {
                if (const llvm::AllocaInst *AI = llvm::dyn_cast<llvm::AllocaInst>(V)) {
                    const llvm::PointerType *PT = AI->getType();
                    if (PT->getPointerElementType()->isIntegerTy()) {
                        return true;
                    }
                }
                return false;
            };
            
            const auto is_gep = [] (const llvm::Value *V) -> bool {
                return llvm::isa<llvm::GetElementPtrInst>(V);
            };
            
            if ((is_integral(V) && is_gep(P)) || (is_integral(P) && is_gep(V))) {
                return llvm::NoAlias;
            }
            
            return alias_result;
        }
            
        default: std::abort();
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
            std::erase_if(out.stores, [P, &AA] (const llvm::Value *V) -> bool {
                return alias(P, V, AA) != llvm::NoAlias;
            });
        } else {
            // erase 'must alias' stores
            std::erase_if(out.stores, [P, &AA] (const llvm::Value *V) -> bool {
                return AA.alias(P, V) == llvm::MustAlias;
            });
            
            // add new store
            out.stores.insert(P);
        }
    } else if (const llvm::LoadInst *LI = llvm::dyn_cast<llvm::LoadInst>(I)) {
        // check if loaded value is ok
        // should find must-alias store
        
        const llvm::Value *L = LI->getPointerOperand();
        
        bool taint = true;
        for (const llvm::Value *S : in.stores) {
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
        bool only_local_stores = false;
        
        if (CI->onlyReadsMemory()) {
            only_local_stores = true;
        } else {
            const llvm::Function *callee = CI->getCalledFunction();
            if (callee && !callee->isDeclaration()) {
                only_local_stores = true;
                for (const llvm::BasicBlock& callee_B : *callee) {
                    for (const llvm::Instruction& callee_I : callee_B) {
                        if (const llvm::StoreInst *callee_SI = llvm::dyn_cast<llvm::StoreInst>(&callee_I)) {
                            if (!llvm::isa<llvm::AllocaInst>(callee_SI->getPointerOperand())) {
                                only_local_stores = false;
                                break;
                            }
                        }
                    }
                }
            }
        }
        
        if (!only_local_stores) {
            out.stores.clear();
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

template <class Func>
void AttackerTaintPass::for_each_instruction(const llvm::Function& F, Func func) {
    for (const llvm::BasicBlock& B : F) {
        for (const llvm::Instruction& I : B) {
            func(&I);
        }
    }
}

bool AttackerTaintResults::get(const llvm::Value *V) const {
    if (const llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(V)) {
        assert(I->getFunction() == F);
        return insts.contains(I);
    } else if (llvm::isa<llvm::Argument>(V)) {
        return true;
    } else if (llvm::isa<llvm::Constant, llvm::BasicBlock>(V)) {
        return false;
    } else {
        llvm::errs() << "unhandled value: " << *V << "\n";
        std::abort();
    }
}

bool AttackerTaintPass::runOnFunction(llvm::Function& F) {
    results = AttackerTaintResults();
    results.F = &F;
    
    llvm::AliasAnalysis AA = std::move(getAnalysis<llvm::AAResultsWrapperPass>().getAAResults());
    
    bool changed;
    Map ins, outs;
    
    // TOP
    Value top {
        Value::Insts(),
        Value::Stores(),
    };
    for_each_instruction(F, [&top] (const llvm::Instruction *I) {
        if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
            const llvm::Value *V = SI->getPointerOperand();
            top.stores.insert(V);
        }
    });
    for_each_instruction(F, [&top, &ins, &outs] (const llvm::Instruction *I) {
        ins.insert_or_assign(I, top);
        outs.insert_or_assign(I, top);
    });
    
    // initialize IN[ENTRY]
    ins.insert_or_assign(&F.getEntryBlock().front(), Value {
        Value::Insts(),
        Value::Stores(),
    });
    
#if CLOU
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
                for (const llvm::Value *V : out.stores) {
                    os << *V << "; ";
                }
                os << "\n";
                os << "OUT: " << desc(outs) << "\n";
            }
        }
        os << "\n\n\n";
        
        // DEBUG: print which stores may alias
        for (auto it1 = top.stores.begin(); it1 != top.stores.end(); ++it1) {
            for (auto it2 = std::next(it1); it2 != top.stores.end(); ++it2) {
                const auto result = alias(*it1, *it2, AA);
                if (result != llvm::NoAlias) {
                    os << **it1 << " - " << **it2 << " : " << result << "\n";
                }
            }
        }
    };
#endif
    
    do {
        changed = false;
        
        for (const llvm::BasicBlock& B : F) {
            for (const llvm::Instruction& I : B) {
                
                /* meet operator */
                {
                    // if block entry
                    if (&I == &B.front()) {
                        
                        auto& in = ins.at(&I);
                        Value new_in = top;
                        for (const llvm::BasicBlock *B_pred : llvm::predecessors(&B)) {
                            const llvm::Instruction *I = B_pred->getTerminator();
                            new_in = meet(new_in, outs.at(I), AA);
                        }
                        if (in != new_in) { changed = true; }
                        in = new_in;
                        
                    } else {
                        
                        const llvm::Instruction *I_prev = I.getPrevNode();
                        const auto& out = outs.at(I_prev);
                        auto& in = ins.at(&I);
                        if (in != out) { changed = true; }
                        ins.at(&I) = outs.at(I_prev);
                        
                    }
                    
                }
                
                /* transfer ins -> outs */
                {
                    auto& out = outs.at(&I);
                    auto new_out = transfer(&I, ins.at(&I), AA);
                    if (out != new_out) { changed = true; }
                    out = new_out;
                }
            }
        }
        
    } while (changed);

#if CLOU
    print();
#endif
    
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

