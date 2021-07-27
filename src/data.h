#pragma once

#include <unordered_map>

#include <llvm/IR/Instruction.h>

#include "dataflow.h"

/* Data dependency analysis */
using DDAValue = std::unordered_map<const llvm::Instruction *,
                                    std::unordered_set<const llvm::Instruction *>
                                    >;

class DataDependencyAnalysis: public DataflowAnalysis<DataDependencyAnalysis,
                                                      DDAValue,
                                                      DA_BACKWARD> {
public:
   using Value = DDAValue;

   void getResult(const llvm::Function& F, BinaryInstRel& addr);

protected:
   void transfer(const llvm::Instruction *inst, const Value& in, Value& out) const;
   void meet(const Value& a, const Value& b, Value& res) const;
   const Value& top() const { return top_; }
   void entry(Value& res) const { res = top(); }

private:
   static inline Value top_ {};
};
