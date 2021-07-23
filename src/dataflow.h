#pragma once

#include <unordered_map>

#include <llvm/IR/Function.h>

#include "util.h"

constexpr bool DA_FORWARD = true;
constexpr bool DA_BACKWARD = false;

template <typename T, typename Value, bool dir>
class DataflowAnalysis {
public:
   using State = std::unordered_map<const llvm::Instruction *, Value>;

   void run(const llvm::Function& F);

   const Value& get_in(const llvm::Instruction& I) const { return in.at(&I); }
   const Value& get_in(const llvm::BasicBlock& B) const { return in.at(&B.front()); }
   const Value& get_out(const llvm::Instruction& I) const { return out.at(&I); }
   const Value& get_out(const llvm::BasicBlock& B) const { return out.at(&B.back()); }
   
protected:
   void transfer_(const llvm::Instruction *inst, const Value& in, Value& out) {
      static_cast<T *>(this)->transfer(inst, in, out);
   }
   void meet_(const Value& a, const Value& b, Value& res) {
      static_cast<T *>(this)->meet(a, b, res);
   }
   const Value& top_() { return static_cast<T *>(this)->top(); }
   void entry_(Value& res) { static_cast<T *>(this)->entry(res); }
   bool direction_() const { return static_cast<const T *>(this)->direction(); }
   
private:
   State in;
   State out;

   const State& din() const { return dir == DA_FORWARD ? in : out; }
   State& din() { return dir == DA_FORWARD ? in : out; }
   const State& dout() const { return dir == DA_FORWARD ? out : in; }
   State& dout() { return dir == DA_FORWARD ? out : in; }
};


template <typename T, typename Value, bool dir>
void DataflowAnalysis<T, Value, dir>::run(const llvm::Function& F) {
   /* Initialize state */
   din().clear();
   for (const llvm::BasicBlock& B : F) {
      for (const llvm::Instruction& I : B) {
         din().emplace(&I, top_());
      }
   }
   entry_(in.at(&F.front().front()));

   /* Construct predecessor map */
   const auto meet_map_f = dir == DA_FORWARD ? predecessor_map : successor_map;
   BinaryInstRel meet_map; 
   meet_map_f(F, meet_map);
   
   predecessor_map(F, meet_map);
   
   /* Do analysis */
   bool changed = true;   
   while (changed) {
      changed = false;
      
      /* 1. Compute OUT from IN */
      for (const llvm::BasicBlock& B : F) {
         for (const llvm::Instruction& I : B) {
            transfer_(&I, din().at(&I), dout()[&I]);
         }
      }

      /* 2. Assign OUT to IN. */
      for (const llvm::BasicBlock& B : F) {
         auto I_it = B.begin();
         assert(I_it != B.end());

         /* Meet predecessors */
         for (auto I_it = B.begin(); I_it != B.end(); ++I_it) {
            const llvm::Instruction *dst = &*I_it;
            Value res = top_();
            for (const llvm::Instruction *src : meet_map[dst]) {
               const Value a = res;
               const Value& b = dout().at(src);
               meet_(a, b, res);
            }

            Value& inval = din().at(dst);
            if (inval != res) {
               changed = true;
            }
            inval = res;
         }
      }
   }
}
