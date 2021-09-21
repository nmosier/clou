#include "taint.h"

void Taint::run() {
    std::vector<NodeRef> order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    
    for (NodeRef ref : order) {
        Node& node = aeg.lookup(ref);
        
        const auto& pred_set = aeg.po.po.rev.at(ref);
        if (pred_set.size() == 1 && false) {
            
            node.taint_mem = aeg.lookup(*pred_set.begin()).taint_mem;
            
        } else {
            
            const z3::sort taint_mem_sort = aeg.context.context.array_sort(aeg.context.context.int_sort(), aeg.context.context.bool_sort());
            std::stringstream ss;
            ss << "taint_mem" << ref;
            node.taint_mem = aeg.context.context.constant(ss.str().c_str(), taint_mem_sort);
            
            std::vector<std::pair<NodeRef, z3::expr>> preds;
            aeg.get_nodes(Direction::IN, ref, std::back_inserter(preds), Edge::PO);
            aeg.get_nodes(Direction::IN, ref, std::back_inserter(preds), Edge::TFO);
            
            
            for (const auto& pred : preds) {
                node.constraints(z3::implies(node.exec() && pred.second, node.taint_mem == aeg.lookup(pred.first).taint_mem), "taint-pred");
            }
            
        }
        
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
    node.taint = aeg.context.FALSE;
    node.taint_mem = z3::const_array(aeg.context.context.int_sort(), aeg.context.TRUE);
}

void Taint::handle_exit(NodeRef ref, Node& node) {
    node.taint = aeg.context.FALSE;
}

void Taint::handle_inst(NodeRef ref, Node& node) {
    z3::expr& taint = node.taint;
    
    switch (node.inst.kind) {
        case Inst::READ:
            taint = node.taint_mem[node.get_memory_address()];
            break;
            
        case Inst::WRITE:
            taint = aeg.context.FALSE;
            node.taint_mem = z3::store(node.taint_mem, node.get_memory_address(), get_value(aeg, ref, node.inst.I->getOperand(0)));
            break;
            
        case Inst::OTHER: {
            /* Definition taint rules for arithmetic (non-memory) instructions.
             * The result is tainted iff any of the input operands are tainted.
             * NOTE: This definition also works as expected for getelementptr instructions.
             */
            taint = aeg.context.FALSE;
            for (const llvm::Value *V : node.inst.I->operand_values()) {
                taint = taint || get_value(aeg, ref, V);
            }
            break;
        }
            
        default: std::abort();
    }
    
    if (!taint.is_false()) {
        const z3::expr var = aeg.context.make_bool("taint");
        node.constraints(z3::implies(node.exec(), var == taint), "taint-def");
        taint = var;
    }
}

z3::expr Taint::get_value(const AEG& aeg, NodeRef ref, const llvm::Value *V) {
    const auto& node = aeg.po.lookup(ref);
    const auto it = node.refs.find(V);
    if (it == node.refs.end()) {
        if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
            return aeg.context.TRUE;
        } else if (const llvm::Constant *C = llvm::dyn_cast<llvm::Constant>(V)) {
            return aeg.context.FALSE;
        } else if (const llvm::BasicBlock *B = llvm::dyn_cast<llvm::BasicBlock>(V)) {
            return aeg.context.FALSE;
        } else {
            llvm::errs() << "error: " << *V << "\n";
            std::abort();
        }
    } else {
        // TODO: This might be a bug, or at least inaccurate.
        z3::expr taint = aeg.context.FALSE;
        for (const NodeRef src : it->second) {
            taint = taint || aeg.lookup(src).taint;
        }
        return taint;
    }
}
