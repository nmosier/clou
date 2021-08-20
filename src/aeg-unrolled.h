#pragma once

#include "aeg-po2.h"
#include "scoped-map.h"

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
      ID id;
      std::vector<FuncID> callstack;
      FuncID next_func;
      LoopID next_loop;

      void push() {
         callstack.push_back(id.func);
         id.func = next_func++;
      }

      void pop() {
         id.func = callstack.back();
         callstack.pop_back();
      }
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

   using ParameterMap = std::unordered_map<std::pair<FuncID, const llvm::Argument *>,
                                           std::pair<FuncID, const llvm::Value *>>;
   ParameterMap params;

   void construct_instruction(const llvm::Instruction *I, Port& port, IDs& ids);
   void construct_call(const llvm::CallBase *C, Port& port, IDs& ids);
   void construct_block(const llvm::BasicBlock *B, Port& port, IDs& ids);
   void construct_loop(const llvm::Loop *L, Port& port, IDs& ids);
   void construct_function(llvm::Function *F, Port& port, IDs& ids);
   void construct_loop_forest(const LoopForest *LF, Port& port, IDs& ids);

   using ArgBinding = std::unordered_map<const llvm::Argument *, NodeRefSet>;
   using InstBinding = std::unordered_map<const llvm::Instruction *, NodeRefSet>;
   struct Binding {
      ArgBinding args;
      InstBinding insts;

      std::optional<NodeRefSet> bind(const llvm::Value *V) const {
         if (const llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(V)) {
            return insts.at(I);
         } else if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
            const auto arg_it = args.find(A);
            if (arg_it == args.end()) {
               return std::nullopt;
            } else {
               return arg_it->second;
            }
         } else {
            return std::nullopt;
         }
      }

      void join(const Binding& other) {
         // join args
         for (const auto& arg : other.args) {
            args[arg.first].insert(arg.second.begin(), arg.second.end());
         }
         // join insts
         for (const auto& inst : other.insts) {
            insts[inst.first].insert(inst.second.begin(), inst.second.end());
         }
      }
   };
   void pass_resolve_addr_refs();

   template <typename InputIt>
   void connect(InputIt src_begin, InputIt src_end, NodeRef dst) {
      for (auto src_it = src_begin; src_it != src_end; ++src_it) {
         add_edge(src_it->second, dst);
      }
   }

   template <typename OutputIt>
   void reverse_postorder(OutputIt out) const;
   bool postorder_rec(NodeRefSet& done, NodeRefVec& order, NodeRef ref) const;
};
