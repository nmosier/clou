#include <llvm/IR/Instructions.h>
#include <cassert>

#include "transient-aa.h"
#include "dataflow.h"

namespace {

struct Value {
    using Pointers = std::unordered_set<const llvm::Value *>;
    Pointers pointers;
    
    static Value transfer(const llvm::Instruction *I, const Value& in);
    static Value meet(const Value& a, const Value& b);
    
    bool operator==(const Value&) const = default;
};

Value Value::transfer(const llvm::Instruction *I, const Value& in) {
    Value out = in;
    
    if (llvm::isa<llvm::LoadInst, llvm::CallBase>(I)) {
        out.pointers.erase(I);
    } else if (llvm::isa<llvm::AllocaInst>(I)) {
        out.pointers.insert(I);
    } else if (const llvm::GetElementPtrInst *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(I)) {
        if (in.pointers.contains(GEP->getPointerOperand()) && std::all_of(GEP->op_begin(), GEP->op_end(), [] (const llvm::Value *op) {
            return llvm::isa<llvm::Constant>(op);
        })) {
            out.pointers.insert(I);
        } else {
            out.pointers.erase(I);
        }
    } else if (const llvm::BitCastInst *BC = llvm::dyn_cast<llvm::BitCastInst>(I)) {
        const llvm::Value *op = BC->getOperand(0);
        if (BC->getType()->isPointerTy() && op->getType()->isPointerTy()) {
            if (in.pointers.contains(op)) {
                out.pointers.insert(BC);
            } else {
                out.pointers.erase(BC);
            }
        }
    } else {
        llvm::errs() << *I << "\n";
        assert(!I->getType()->isPointerTy());
    }
    
    return out;
}

Value Value::meet(const Value& a, const Value& b) {
    Value out;
    
    // intersect
    std::copy_if(a.pointers.begin(), a.pointers.end(), std::inserter(out.pointers, out.pointers.end()), [&b] (const llvm::Value *V) -> bool {
        return b.pointers.contains(V);
    });
    
    return out;
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const Value& value) {
    os << "{";
    for (bool first = true; const llvm::Value *pointer : value.pointers) {
        if (!first) {
            os << "; ";
        }
        first = false;
        os << *pointer;
    }
    os << "}";
    return os;
}

}

TransientAAResults::TransientAAResults(const llvm::Function& F) {
    using DF = dataflow::Dataflow<Value>;
    
    // get top
    Value top;
    for (const llvm::BasicBlock& B : F) {
        for (const llvm::Instruction& I : B) {
            if (I.getType()->isPointerTy()) {
                top.pointers.insert(&I);
            }
        }
    }
    
    DF::Context context = {
        .top = top,
        .transfer = &Value::transfer,
        .meet = &Value::meet,
    };
    DF::Function function (context, &F, nullptr, DF::Function::Mode::INST);
    DF::Map ins, outs, exit_values;
    function.transfer(top, ins, outs, exit_values);
    
    // get results: take intersection of all outs
    valid = top.pointers;
    for (const auto& p : outs) {
        std::erase_if(valid, [&p] (const llvm::Value *pointer) {
            return !p.second.pointers.contains(pointer);
        });
    }
    
    // DEBUG
    llvm::errs() << "Transient AA Results:\n";
    for (const llvm::Value *pointer : valid) {
        llvm::errs() << "  " << *pointer << "\n";
    }
}
