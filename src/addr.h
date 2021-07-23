#pragma once

#include <unordered_set>
#include <unordered_map>

#include <llvm/IR/Instruction.h>

#include "dataflow.h"

using ADAValue = std::unordered_map<const llvm::Instruction *,
                                    std::unordered_set<const llvm::Instruction *>
                                    >;

/* This maps load sources to the set of values that the load's result may be used to compute.
 */
class AddressDependencyAnalysis: public DataflowAnalysis<AddressDependencyAnalysis, ADAValue,
                                                         DA_FORWARD> {
public:
   using Value = ADAValue;
   
   void getResult(const llvm::Function& F, BinaryInstRel& addr) const;
   
protected:
   void transfer(const llvm::Instruction *inst, const Value& in, Value& out) const;
   void meet(const Value& a, const Value& b, Value& res) const;
   const Value& top() const { return top_; }
   void entry(Value& res) const { res = top(); }
   
private:
   static inline Value top_ {};
   template <typename, typename, bool> friend class DataflowAnalysis;
};


