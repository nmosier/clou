#pragma once

#include <string>

#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>

#include "binrel.h"
#include "util.h"

#define CFG CFG2

class CFG {
public:   
   using Rel = binrel<const llvm::Instruction *>;
   Rel po;

   void construct(const llvm::Function& F);

   void dump_graph(const std::string& path) const {
      po.group().dump_graph(path, [] (llvm::raw_ostream& os, const auto& bb) {
         for (const llvm::Instruction *I : bb) {
            if (I) {
               os << *I;
            } else {
               os << "<ENTRY/EXIT>";
            }
            os << "\n";
         }
      });
   }

   struct CallSite {
      using Set = std::unordered_set<const llvm::Instruction *>;
      const llvm::Function& F;
      Set entry;
      Set exit;
      friend struct std::hash<CallSite>;
      bool operator==(const CallSite& other) const {
         return &F == &other.F && entry == other.entry && exit == other.exit;
      }
   };   

private:
   using CallSites = std::unordered_set<CallSite>;
   void construct(const CallSite& site, CallSites& sites);
   void remove_calls();
};

namespace std {
   template <> 
   struct hash<CFG::CallSite> {
      size_t operator()(const CFG::CallSite& site) const {
         return hash_ordered_tuple(&site.F, site.entry, site.exit);
      }
   };
}

#undef CFG
