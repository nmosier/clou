#pragma once

#include "aeg-po2.h"

class AEGPO_Unrolled: public AEGPO2 {
public:
   explicit AEGPO_Unrolled(llvm::Function& F, unsigned num_unrolls = 2):
      F(F),
      num_unrolls(num_unrolls) {
      if (num_unrolls == 0) {
         throw std::invalid_argument("positive number of loop unrolls required");
      }
   }

   // TODO: these should be private
   llvm::Function& F;
   const unsigned num_unrolls;   

   void construct();
   
private:
   struct IDs {
      unsigned func_id = 0;
      unsigned next_loop_id = 0;
      std::vector<unsigned> loop_ids;
   };

   struct Port {
      NodeRef entry;
      std::unordered_multimap<const llvm::BasicBlock *, NodeRef> exits;
   };

   struct LoopForest {
      const llvm::BasicBlock *entry;
      std::vector<const llvm::BasicBlock *> exits;
      std::vector<const llvm::BasicBlock *> blocks;
      std::vector<const llvm::Loop *> loops;
   };   

   void construct_instruction(const llvm::Instruction *I, Port& port, IDs& ids);
   void construct_call(const llvm::CallBase *C, Port& port, IDs& ids);
   void construct_block(const llvm::BasicBlock *B, Port& port, IDs& ids);
   void construct_loop(const llvm::Loop *L, Port& port, IDs& ids);
   void construct_function(llvm::Function *F, Port& port, IDs& ids);
   void construct_loop_forest(const LoopForest *LF, Port& port, IDs& ids);

   template <typename InputIt>
   void connect(InputIt src_begin, InputIt src_end, NodeRef dst) {
      for (auto src_it = src_begin; src_it != src_end; ++src_it) {
         add_edge(src_it->second, dst);
      }
   }
};
