#pragma once

#include <unordered_set>
#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Function.h>

struct TransientAAResults {
    /// Instructions for which architectural alias analysis is still valid under transient execution.
    /// NOTE: In order to use architectural AA results, BOTH values in the query must be valid.
    std::unordered_set<const llvm::Value *> valid;
    
    bool get(const llvm::Value *V) const { return valid.contains(V); }
    
    TransientAAResults() = default;
    TransientAAResults(const llvm::Function& F);
};

