#pragma once

#include <unordered_map>
#include <llvm/IR/Argument.h>
#include <z3++.h>

#include "aeg.h"

class Taint {
public:
    Taint(AEG& aeg): aeg(aeg) {}
    
    void run();
    
    auto operator()() { return run(); }
    
    static z3::expr get_value(const AEG& aeg, NodeRef ref, const llvm::Value *V);
    
private:
    using Node = AEG::Node;
    using Edge = AEG::Edge;
    AEG& aeg;
    std::unordered_map<const llvm::Argument *, z3::expr> args;
    
    void handle_entry(NodeRef ref, Node& node);
    void handle_exit(NodeRef ref, Node& node);
    void handle_inst(NodeRef ref, Node& node);
};
