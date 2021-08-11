#pragma once

/* inst.h -- get information about instructions
 */

#include <llvm/IR/Instructions.h>

#include "xmacro.h"
#include "cfg.h"

struct Inst {

#define INST_KIND_X(X)                          \
   X(ENTRY)                                     \
   X(EXIT)                                      \
   X(READ)                                      \
   X(WRITE)                                     \
   X(FENCE)                                     \
   X(OTHER)
   
   XM_ENUM_DEF(Kind, INST_KIND_X);

   const llvm::Instruction *I;
   Kind kind;
   const llvm::Value *addr; // TODO: This should be able to hold multiple addresses.

   static const char *kind_tostr(Kind kind);
   const char *kind_tostr() const { return kind_tostr(kind); }
   void set_kind();
   void set_addr();

   void set_base();
   void set(const llvm::Instruction *I_) { I = I_; set_base(); }
   void set(CFG::Entry) { kind = Kind::ENTRY; set_base(); }
   void set(CFG::Exit) { kind = Kind::EXIT; set_base(); }
   
   template <typename... Args>
   Inst(Args&&... args) { set(std::forward<Args>(args)...); }
};
