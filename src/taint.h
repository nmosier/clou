#pragma once

#include <unordered_map>
#include <llvm/IR/Argument.h>
#include <z3++.h>

#include "noderef.h"

class Taint {
public:
    virtual void run() = 0;
    virtual z3::expr flag(NodeRef ref) = 0;
    virtual ~Taint() {}
};
