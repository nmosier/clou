#include <vector>
#include <unordered_map>

#include "aeg.h"
#include "mcfg.h"

const char *UHBEdge::kind_tostr(Kind kind) {
#define UHBEDGE_KIND_CASE(name) case name: return #name;
   switch (kind) {
      UHBEDGE_KIND_X(UHBEDGE_KIND_CASE)
   default: return nullptr;
   }
#undef UHBEDGE_KIND_CASE
}

UHBEdge::Kind UHBEdge::kind_fromstr(const std::string& s) {
#define UHBEDGE_KIND_PAIR(name) {#name, name}, 
   static const std::unordered_map<std::string, Kind> map {
      UHBEDGE_KIND_X(UHBEDGE_KIND_PAIR)
   };
   return map.at(s);
#undef UHBEDGE_KIND_PAIR
}

void AEG::construct(const MemoryCFG& mcfg) {
   construct(mcfg, nullptr);
}

void AEG::construct(const MemoryCFG& mcfg, const llvm::Instruction *I) {
   if (I == nullptr) { return; }
   
   const auto& succs = mcfg.graph().at(I);

   /* For now, assume that each node in the CFG has at most two outgoing edges (i.e. is a branch).
    * Later we will handle switches.
    */
   const size_t nsuccs = succs.size();
   if (nsuccs > 1) {
      assert(nsuccs == 2);

      for (const llvm::Instruction *succ : succs) {
         /* correct speculation arm */
         
         
         /* misspeculation arm */
         // do simple loop?
         // what about sub branches?
         // Could just include all in a blob
         //
         // 
         
      }
   }

   
   
}
