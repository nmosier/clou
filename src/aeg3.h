#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <llvm/IR/Instruction.h>

#include "util.h"

#define AEG AEG3

class AEG {
public:
   struct Edge {};
   
   using NodeSet = std::unordered_set<Node *>;
   using Rel = std::unordered_map<Node *, NodeSet>;
   using Rels = std::unordered_map<Edge, Rel>;
   
   
private:
};
