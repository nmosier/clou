#include "inst.h"

const char *Inst::kind_tostr(Kind kind) {
   XM_ENUM_TOSTR(INST_KIND_X, kind, "(invalid)");
}

void Inst::set_kind() {
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
}

void Inst::set_addr() {
   switch (kind) {
   case Kind::READ:
      assert(I->getNumOperands() == 1);
      addr = I->getOperand(0);
      break;
   case Kind::WRITE:
      assert(I->getNumOperands() == 2);
      addr = I->getOperand(1);
      break;
   case Kind::FENCE:
   case Kind::OTHER:
   case Kind::ENTRY:
   case Kind::EXIT:
      addr = nullptr;
      break;
   default: std::abort();
   }
   if (addr) {
      assert(addr->getType()->isPointerTy());
   }
}

void Inst::set_base() {
   set_kind();
   set_addr();
}
