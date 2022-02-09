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
#include <unordered_set>
#include <fstream>
#include <gperftools/profiler.h>


#include "attacker-taint.h"
#include "annotations.h"
#if CLOU
# include "config.h"
#endif

#include "dataflow/dataflow.h"

struct Value {
    using Insts = std::unordered_set<const llvm::Instruction *>;
    using Stores = std::unordered_set<const llvm::Value *>;
    
    /** Attacker-controlled instructions. */
    Insts insts;

    /** Non-attacker-controlled stores. */
    Stores stores;
    
    bool operator==(const Value& other) const = default;
    
    Value() = delete;
    Value(const Insts& insts, const Stores& stores): insts(insts), stores(stores) {}
};
using Map = std::unordered_map<const llvm::Instruction *, Value>;

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const Value& value) {
    os << "Insts:\n";
    for (const llvm::Instruction *I : value.insts) {
        os << "  " << *I << "\n";
    }
    os << "Stores:\n";
    for (const llvm::Value *V : value.stores) {
        os << "  " << *V << "\n";
    }
    return os;
}

Value meet(const Value& a, const Value& b, llvm::AliasAnalysis& AA) {
    Value out {Value::Insts(), Value::Stores()};
    
    // union controlled instructions
    std::copy(a.insts.begin(), a.insts.end(), std::inserter(out.insts, out.insts.end()));
    std::copy(b.insts.begin(), b.insts.end(), std::inserter(out.insts, out.insts.end()));
    
    // NOTE: This could be replaced by a bitwise OR of std::vector<bool>.
    // Could map stores to group of stores that must alias?
    
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

Value transfer(const llvm::Instruction *I, const Value& in, llvm::AliasAnalysis& AA, const std::unordered_set<const llvm::Value *>& annotated_uncontrolled) {
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
        /* NOTE: It doesn't matter if the pointer operand is arch-controlled, since we are assuming there are no architectural OOB accesses (i.e. no bugs). */
        
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
    
    std::copy(annotated_uncontrolled.begin(), annotated_uncontrolled.end(), std::inserter(out.stores, out.stores.end()));
    
    return out;
}

void AttackerTaintPass::getAnalysisUsage(llvm::AnalysisUsage& AU) const {
    AU.addRequired<llvm::AAResultsWrapperPass>();
    AU.addRequired<llvm::LoopInfoWrapperPass>();
    // AU.addRequired<AnnotationPass>();
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
    
    llvm::AliasAnalysis& AA = getAnalysis<llvm::AAResultsWrapperPass>().getAAResults();
    llvm::LoopInfo& LI = getAnalysis<llvm::LoopInfoWrapperPass>().getLoopInfo();
    // const Annotations& annotations = getAnalysis<AnnotationPass>().getResults();
    Annotations annotations;
    llvm::parse_annotations(*F.getParent(), std::inserter(annotations, annotations.end()));
    
    // get clou.arch-uncontrolled annotations
    std::unordered_set<const llvm::Value *> annotated_uncontrolled;
    for (const auto& p : annotations) {
        if (p.second == "clou.arch_uncontrolled") {
            annotated_uncontrolled.insert(p.first);
        }
    }
    
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
    
    const llvm::Instruction *entry = &F.getEntryBlock().front();
    
    // initialize IN[ENTRY]
    ins.at(entry) = Value {
        Value::Insts(),
        Value::Stores(),
    };
    
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
                if (&I != entry) {
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
                    auto new_out = transfer(&I, ins.at(&I), AA, annotated_uncontrolled);
                    if (out != new_out) { changed = true; }
                    out = new_out;
                }
            }
        }
        
    } while (changed);

#if CLOU
    print();
#endif
    
    if (true) {
        
        // test using dataflow
        using Dataflow = dataflow::Dataflow<Value>;
        Dataflow::Context context = {
            .top = top,
            .transfer = [&AA, &annotated_uncontrolled] (const llvm::Instruction *I, const Value& in) -> Value {
                return transfer(I, in, AA, annotated_uncontrolled);
            },
                .meet = [&AA] (const Value& a, const Value& b) -> Value {
                    return meet(a, b, AA);
                },
        };
        Map ins2;
        Map outs2;
        Map exit_values;
#if 0
        Dataflow::Graph graph = Dataflow::Graph::from_function_block(F, context);
        graph.transfer(Value {Value::Insts(), Value::Stores()}, ins2, outs2, exit_values);
#else
        Dataflow::Function function {context, &F, &LI, Dataflow::Function::Mode::LOOP};
        function.transfer(Value {Value::Insts(), Value::Stores()}, ins2, outs2, exit_values);
#endif
        
        // DEBUG
        
        // check if keys same at least
        const auto keys_subset = [] (const auto& a, const auto& b) -> bool {
            return std::all_of(a.begin(), a.end(), [&] (const auto& p) -> bool {
                return b.contains(p.first);
            });
        };
        assert(keys_subset(ins, ins2));
        assert(keys_subset(ins2, ins));
        
        const auto print_diffs = [] (const Map& a, const Map& b) {
            llvm::errs() << "Differences:\n\n";
            for (const auto& p : a) {
                const auto& key = p.first;
                if (a.at(key) != b.at(key)) {
                    llvm::errs() << *key << ":\n" << a.at(key) << "\n" << b.at(key) << "\n\n";
                }
            }
        };
        
        print_diffs(ins, ins2);
        
#if 0
        assert(ins == ins2);
        assert(outs == outs2);
#else
        if (ins != ins2) {
            llvm::errs() << "WARNING: ins != ins2: " << __FILE__ << ":" << __LINE__ << "\n";
        }
        if (outs != outs2) {
            llvm::errs() << "WARNING: outs != outs2: " << __FILE__ << ":" << __LINE__ << "\n";
        }
        ins = ins2;
        outs = outs2;
#endif
    }
    
    
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

void AttackerTaintPass::print(llvm::raw_ostream& os, const llvm::Module *M) const {
    os << results << "\n";
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


llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AttackerTaintResults& results) {
    os << "Controlled Instructions:\n";
    for (const llvm::Instruction *I : results.insts) {
        os << "  " << *I << "\n";
    }
    return os;
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
