#pragma once

#include "aeg/aeg.h"
#include "taint.h"

class Taint_BV: public Taint {
public:
    static constexpr unsigned taint_bits = 2;
    Taint_BV(aeg::AEG& aeg): aeg(aeg), ctx(aeg.context.context) {}
    
    virtual z3::expr flag(NodeRef ref) override {
        const Node& node = aeg.lookup(ref);
#if 0
        assert(node.is_memory_op());
#else
        todo();
#endif
        return get_value(ref, node.get_memory_address_pair().first);
    }
    
    virtual z3::expr get_value(NodeRef ref, const llvm::Value *V) const override;
    
protected:
    using Node = aeg::AEG::Node;
    using Edge = aeg::AEG::Edge;
    aeg::AEG& aeg;
    z3::context& ctx;
    std::unordered_map<const llvm::Argument *, z3::expr> args;
};

class Taint_Array: public Taint_BV {
public:
    z3::expr taint_mem;
    Taint_Array(aeg::AEG& aeg): Taint_BV(aeg), taint_mem(aeg.context) {}
    
    virtual void run() override;
    
private:
    void handle_entry(NodeRef ref, Node& node);
    void handle_exit(NodeRef ref, Node& node);
    void handle_inst(NodeRef ref, Node& node);
};
