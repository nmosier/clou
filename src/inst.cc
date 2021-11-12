#include <iostream>

#include "inst.h"
#include "util/llvm.h"
#include "cfg/cfg.h"

Inst *Inst::Create(Entry) {
    return new EntryInst();
}

Inst *Inst::Create(Exit) {
    return new ExitInst();
}

Inst *Inst::Create(const llvm::Instruction *I) {
    if (llvm::isa<llvm::FenceInst>(I)) {
        return new FenceInst(I);
    } else if (llvm::isa<llvm::LoadInst>(I)) {
        return new LoadInst(I);
    } else if (llvm::isa<llvm::StoreInst>(I)) {
        return new StoreInst(I);
    } else {
        if (I->mayReadOrWriteMemory()) {
            llvm::errs() << *I << "\n";
        }
        
        // DEBUG assertions
        {
            if (I->mayReadOrWriteMemory()) {
                if (const auto *C = llvm::dyn_cast<llvm::CallInst>(I)) {
                    if (C->getNumArgOperands() != 0) {
                        llvm::errs() << *C << "\n";
                    }
                    assert(C->getNumArgOperands() == 0);
                } else {
                    assert(false);
                }
            }
        }

        return new OtherInst(I);
    }
}

Inst *Inst::Create(const CFG::Node::Call& call) {
    return new CallInst(call.C, call.arg);
}

RegularInst::RegularInst(const llvm::Instruction *I): I(I) {
    if (I->getType()->isPointerTy()) {
       addr_def = static_cast<const llvm::Value *>(I);
    }
    
    // check for addr refs
    for (const llvm::Value *V : I->operand_values()) {
       if (V->getType()->isPointerTy()) {
          addr_refs.push_back(V);
       }
    }
}

CallInst::CallInst(const llvm::Instruction *I, const llvm::Value *arg): MemoryInst(I), arg(arg) {
    addr_refs = {arg};
}

void Inst::print(std::ostream& os) const {
    os << kind();
}
