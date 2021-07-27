#pragma once

#include <unordered_set>
#include <unordered_map>

class BinaryInstRel: public std::unordered_map<const llvm::Instruction *,
                                               std::unordered_set<const llvm::Instruction *>
                                               > {
public:
   using Nodes = std::unordered_set<const llvm::Instruction *>;

   Nodes nodes() const;
   void pred_map(const llvm::Function& F);
   void succ_map(const llvm::Function& F);
   void prune();
   void remove_node(const llvm::Instruction *inst);

   
private:
};
