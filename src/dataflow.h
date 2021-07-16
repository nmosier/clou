#pragma once

#include <unordered_map>

#include <llvm/IR/Function.h>

#include "util.h"

template <typename Value>
class DataflowAnalysis {
public:
   using State = std::unordered_map<const llvm::Instruction *, Value>;

   void run(const llvm::Function& F);

   const Value& get_in(const llvm::Instruction& I) const { return in.at(&I); }
   const Value& get_in(const llvm::BasicBlock& B) const { return in.at(&B.front()); }
   const Value& get_out(const llvm::Instruction& I) const { return out.at(&I); }
   const Value& get_out(const llvm::BasicBlock& B) const { return out.at(&B.back()); }
   
protected:
   virtual void transfer(const llvm::Instruction *inst, const Value& in, Value& out) = 0;
   virtual void meet(const Value& a, const Value& b, Value& res) = 0;
   virtual const Value& top() = 0;
   virtual void entry(Value& res) = 0;
   
private:
   State in;
   State out;
};


template <typename Value>
void DataflowAnalysis<Value>::run(const llvm::Function& F) {
   /* Initialize state */
   in.clear();
   for (const llvm::BasicBlock& B : F) {
      for (const llvm::Instruction& I : B) {
         in.emplace(&I, top());
      }
   }
   entry(in.at(&F.front().front()));

   /* Construct predecessor map */
   PredecessorMap preds;
   predecessor_map(F, preds);
   
   /* Do analysis */
   bool changed = true;   
   while (changed) {
      changed = false;
      
      /* 1. Compute OUT from IN */
      for (const llvm::BasicBlock& B : F) {
         for (const llvm::Instruction& I : B) {
            transfer(&I, in.at(&I), out[&I]);
         }
      }

      /* 2. Assign OUT to IN. */
      for (const llvm::BasicBlock& B : F) {
         auto I_it = B.begin();
         assert(I_it != B.end());

         /* Meet predecessors */
         for (auto I_it = B.begin(); I_it != B.end(); ++I_it) {
            const llvm::Instruction *dst = &*I_it;
            Value res = top();
            for (const llvm::Instruction *src : preds[dst]) {
               const Value a = res;
               const Value& b = out.at(src);
               meet(a, b, res);
            }

            Value& inval = in.at(dst);
            if (inval != res) {
               changed = true;
            }
            inval = res;
         }
      }
   }
}
