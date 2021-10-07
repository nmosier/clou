#include <iostream>

#include "inst.h"
#include "util/llvm.h"
#include "util.h"
#include "aeg-po2.h"

#if 0
const char *Inst::kind_tostr(Kind kind) {
   XM_ENUM_CLASS_TOSTR(INST_KIND_X, Kind, kind, "(invalid)");
}
#endif

#if 0
void Inst::set_kind() {
    switch (I->getOpcode()) {
        case llvm::Instruction::Call:
            kind = Kind::OTHER;
            break;
        default: {
            const bool read = I->mayReadFromMemory();
            const bool write = I->mayWriteToMemory();
            assert(!(read && write)); // LLVM-IR shouldn't allow this.
            if (read) {
               kind = Kind::READ;
            } else if (write) {
               kind = Kind::WRITE;
            } else if (llvm::isa<llvm::FenceInst>(I)) {
               kind = Kind::FENCE;
            } else if (llvm::isa<llvm::CallBase>(I)) {
                kind = Kind::CALL;
            } else {
               kind = Kind::OTHER;
            }
            break;
        }
    }
}
#endif

#if 0
void Inst::set_addr() {
   switch (kind) {
   case Kind::ENTRY:
   case Kind::EXIT:
      break;
      
   default:
      // check for addr def
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
}
#endif

#if 0
std::ostream& operator<<(std::ostream& os, Inst::Kind kind) {
    return os << Inst::kind_tostr(kind);
}

std::ostream& operator<<(std::ostream& os, const Inst& inst) {
    os << inst.kind;
   switch (inst.kind) {
   case Inst::Kind::ENTRY:
   case Inst::Kind::EXIT:
      break;
   default:
      os << " " << *inst.I;
   }
   return os;
}
#endif

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
        assert(!I->mayReadOrWriteMemory());
        return new OtherInst(I);
    }
}

Inst *Inst::Create(const AEGPO::Node::Call& call) {
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
