#pragma once

#include <unordered_set>

#include "dataflow.h"

/* This maps load sources to the set of values that the load's result may be used to compute.
 */
using AddressDependencyAnalysisValue = std::unordered_map<
   const llvm::Instruction *,
   std::unordered_set<const llvm::Instruction *>
   >;

class AddressDependencyAnalysis: public DataflowAnalysis<AddressDependencyAnalysisValue> {
public:
   using Value = AddressDependencyAnalysisValue;
protected:
   virtual void transfer(const llvm::Instruction *inst, const Value& in, Value& out) override;
   virtual void meet(const Value& a, const Value& b, Value& res) override;
   virtual const Value& top() override { return top_; }
   virtual void entry(Value& res) override { res = top(); }
   
private:
   static inline Value top_ {};
};
