#pragma once

#include "aeg.h"
#include "taint.h"

class Taint_BV: public Taint {
public:
    static constexpr unsigned taint_bits = 2;
    Taint_BV(AEG& aeg): aeg(aeg), ctx(aeg.context.context), bot(ctx.bv_val(0, taint_bits)), mid(ctx.bv_val(1, taint_bits)), top(ctx.bv_val(3, taint_bits)), taint_sort(ctx.bv_sort(2)) {}
    
    virtual z3::expr flag(NodeRef ref) override {
        const Node& node = aeg.lookup(ref);
        return get_value(ref, node.get_memory_address_pair().first).extract(1, 1) == ctx.bv_val(1, 1);
    }
    
protected:
    using Node = AEG::Node;
    using Edge = AEG::Edge;
    AEG& aeg;
    z3::context& ctx;
    std::unordered_map<const llvm::Argument *, z3::expr> args;
    const z3::expr bot, mid, top;
    const z3::sort taint_sort;
    
    virtual z3::expr get_value(NodeRef ref, const llvm::Value *V) const;
};

class Taint_Array: public Taint_BV {
public:
    Taint_Array(AEG& aeg): Taint_BV(aeg), taint_mem_sort(ctx.array_sort(ctx.int_sort(), taint_sort)) {}
    
    virtual void run() override;
    
private:
    const z3::sort taint_mem_sort;

    void handle_entry(NodeRef ref, Node& node);
    void handle_exit(NodeRef ref, Node& node);
    void handle_inst(NodeRef ref, Node& node);
};

class Taint_Function: public Taint_BV {
public:
    Taint_Function(AEG& aeg);
    
    virtual void run() override;

private:
    z3::expr taint_mem;
};
