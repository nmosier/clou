#include <llvm/IR/Instructions.h>

#include "cfg.h"

#define CFG CFG2

void CFG::construct(const llvm::Function& F) {
   CallSites sites;
   const CallSite site {F, {nullptr}, {nullptr}};
   construct(site, sites);
}

void CFG::construct(const CallSite& site, CallSites& seen) {
   // check if call site already seen; otherwise mark as seen
   const auto seen_res = seen.insert(site);
   if (!seen_res.second) { return; }
   
   po.insert(site.entry.begin(), site.entry.end(), &site.F.front().front());
   
   for (const llvm::BasicBlock& B : site.F) {
      /* add linear body of BB */
      for (auto cur = B.begin(), next = std::next(cur); next != B.end(); cur = next, ++next) {
         po.insert(&*cur, &*next);
      }

      /* add terminator */
      const llvm::Instruction *term = &B.back();
      const auto num_succs = term->getNumSuccessors();
      if (num_succs == 0) {
         po.insert(term, site.exit.begin(), site.exit.end());
      } else {
         for (size_t i = 0; i < num_succs; ++i) {
            const llvm::Instruction *succ = &term->getSuccessor(i)->front();
            po.insert(term, succ);
         }
      }
   }

   /* process calls */
   for (const llvm::BasicBlock& B : site.F) {
      for (const llvm::Instruction& I : B) {
         if (const auto *C = llvm::dyn_cast<llvm::CallBase>(&I)) {
            const CallSite newsite {*C->getCalledFunction(), po.rev.at(C), po.fwd.at(C)};
            construct(newsite, seen);
         }
      }
   }
}


#undef CFG
