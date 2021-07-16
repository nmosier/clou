#pragma once

#include <unordered_map>
#include <unordered_set>
#include <string>

#include <llvm/IR/Function.h>

using PredecessorMap = std::unordered_map<
   const llvm::Instruction *,
   std::vector<const llvm::Instruction *>
   >;

void predecessor_map(const llvm::Function& F, PredecessorMap& preds);

using BinaryInstRel = std::unordered_map<const llvm::Instruction *,
                                         std::unordered_set<const llvm::Instruction *>
                                         >;
