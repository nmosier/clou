#pragma once

/* inst.h -- get information about instructions
 */

#include <llvm/IR/Instructions.h>

#include "xmacro.h"
#include "lcm.h"

struct Inst {

#define INST_KIND_X(X)                          \
   X(ENTRY)                                     \
   X(EXIT)                                      \
   X(READ)                                      \
   X(WRITE)                                     \
   X(FENCE)                                     \
   X(OTHER)
   
   XM_ENUM_DEF(Kind, INST_KIND_X);

   const llvm::Instruction *I = nullptr;
   Kind kind;
   const llvm::Value *addr = nullptr; // TODO: This should be able to hold multiple addresses.

   static const char *kind_tostr(Kind kind);
   const char *kind_tostr() const { return kind_tostr(kind); }
   void set_kind();
   void set_addr();
   
   void set_base();
   void set(const llvm::Instruction *I_) { I = I_; set_kind(); set_addr(); }
   void set(Entry) { kind = Kind::ENTRY; }
   void set(Exit) { kind = Kind::EXIT; }

   template <typename Arg>
   Inst(const Arg& arg) { set(arg); }
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const Inst& inst);
