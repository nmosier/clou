#pragma once

#include <unordered_map>
#include <llvm/IR/Argument.h>
#include <z3++.h>

#include "aeg.h"

class Taint {
public:
    static constexpr unsigned taint_bits = 2;
    
    Taint(AEG& aeg): aeg(aeg), ctx(aeg.context.context), taint_sort(ctx.bv_sort(taint_bits)), taint_mem_sort(ctx.array_sort(ctx.int_sort(), taint_sort)), bot(ctx.bv_val(0, taint_bits)), mid(ctx.bv_val(1, taint_bits)), top(ctx.bv_val(3, taint_bits)) {}
    
    void run();
    
    auto operator()() { return run(); }
    
    z3::expr get_value(NodeRef ref, const llvm::Value *V) const;
    z3::expr flag(NodeRef ref) {
        const Node& node = aeg.lookup(ref);
        return get_value(ref, node.get_memory_address_pair().first).extract(1, 1) == aeg.context.context.bv_val(1, 1);
    }
    
private:
    using Node = AEG::Node;
    using Edge = AEG::Edge;
    AEG& aeg;
    z3::context& ctx;
    std::unordered_map<const llvm::Argument *, z3::expr> args;
    const z3::sort taint_sort;
    const z3::sort taint_mem_sort;
    
public:
    const z3::expr bot, mid, top;
    
private:
    void handle_entry(NodeRef ref, Node& node);
    void handle_exit(NodeRef ref, Node& node);
    void handle_inst(NodeRef ref, Node& node);
};
