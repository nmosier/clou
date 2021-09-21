#include "taint.h"

void Taint::run() {
    std::vector<NodeRef> order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    
    for (NodeRef ref : order) {
        Node& node = aeg.lookup(ref);
        
        node.taint_mem = ctx.constant((std::string("taint_mem") + std::to_string(ref)).c_str(), taint_mem_sort);
        
        std::vector<std::pair<NodeRef, z3::expr>> preds;
        aeg.get_nodes(Direction::IN, ref, std::back_inserter(preds), Edge::PO);
        aeg.get_nodes(Direction::IN, ref, std::back_inserter(preds), Edge::TFO);
        
        
        for (const auto& pred : preds) {
            node.constraints(z3::implies(node.exec() && pred.second, node.taint_mem == aeg.lookup(pred.first).taint_mem), "taint-pred");
        }
        
        node.taint = bot;
        
        if (node.inst.kind == Inst::ENTRY) {
            handle_entry(ref, node);
        } else if (node.inst.kind == Inst::EXIT) {
            handle_exit(ref, node);
        } else {
            handle_inst(ref, node);
        }
        
    }
}


void Taint::handle_entry(NodeRef ref, Node& node) {
    node.taint_mem = z3::const_array(aeg.context.context.int_sort(), ctx.bv_val(1, 2));
}

void Taint::handle_exit(NodeRef ref, Node& node) {}

void Taint::handle_inst(NodeRef ref, Node& node) {
    z3::expr& taint = node.taint;
    
    switch (node.inst.kind) {
        case Inst::READ: {
            /* A load from an untainted address gets the same taint level as the loaded value.
             * A load from a tainted address always gets taint level 2.
             */
            const z3::expr addr_taint = get_value(ref, node.get_memory_address_pair().first);
            const z3::expr addr_taint_nonzero = z3::bvredor(addr_taint) != ctx.bv_val(0, 1);
            taint = z3::ite(addr_taint_nonzero, top, node.taint_mem[node.get_memory_address()]);
            break;
        }
            
        case Inst::WRITE:
            node.taint_mem = z3::store(node.taint_mem, node.get_memory_address(), get_value(ref, node.inst.I->getOperand(0)));
            break;
            
        case Inst::OTHER: {
            /* Definition taint rules for arithmetic (non-memory) instructions.
             * The result is tainted iff any of the input operands are tainted.
             * NOTE: This definition also works as expected for getelementptr instructions.
             */
            for (const llvm::Value *V : node.inst.I->operand_values()) {
                taint = taint | get_value(ref, V);
            }
            break;
        }
            
        default: std::abort();
    }
    
    if (!taint.is_const()) {
        const z3::expr var = aeg.context.make_const("taint", taint_sort);
        node.constraints(z3::implies(node.exec(), var == taint), "taint-def");
        taint = var;
    }
}

z3::expr Taint::get_value(NodeRef ref, const llvm::Value *V) const {
    const auto& node = aeg.po.lookup(ref);
    const auto it = node.refs.find(V);
    if (it == node.refs.end()) {
        if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
            return mid;
        } else if (const llvm::Constant *C = llvm::dyn_cast<llvm::Constant>(V)) {
            return bot;
        } else if (const llvm::BasicBlock *B = llvm::dyn_cast<llvm::BasicBlock>(V)) {
            return bot;
        } else {
            llvm::errs() << "error: " << *V << "\n";
            std::abort();
        }
    } else {
        // TODO: This might be a bug, or at least inaccurate.
        z3::expr taint = bot;
        for (const NodeRef src : it->second) {
            taint = taint | aeg.lookup(src).taint;
        }
        return taint;
    }
}
