#include <iostream>

#include "inst.h"
#include "llvm-util.h"

const char *Inst::kind_tostr(Kind kind) {
   XM_ENUM_TOSTR(INST_KIND_X, kind, "(invalid)");
}

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
            } else if (I->getOpcode() == llvm::Instruction::Fence) {
               kind = Kind::FENCE;
            } else {
               kind = Kind::OTHER;
            }
            break;
        }
    }
}

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

std::ostream& operator<<(std::ostream& os, const Inst& inst) {
   os << inst.kind_tostr();
   switch (inst.kind) {
   case Inst::Kind::ENTRY:
   case Inst::Kind::EXIT:
      break;
   default:
      os << " " << *inst.I;
   }
   return os;
}
