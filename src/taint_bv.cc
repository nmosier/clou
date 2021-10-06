#include "taint_bv.h"

void Taint_Array::run() {
#if USE_TAINT
        std::vector<NodeRef> order;
        aeg.po.reverse_postorder(std::back_inserter(order));
        
        const z3::sort taint_mem_sort = ctx.array_sort(ctx.int_sort(), ctx.bv_sort(2));
        
        taint_mem = z3::const_array(ctx.int_sort(), ctx.bv_val(3, 2));
        
        const auto read = [&] (const z3::expr& addr, const llvm::Type *T) {
            const z3::expr raw = taint_mem[addr];
            return z3::ite(raw == ctx.bv_val(3, 2),
                           ctx.bool_val(!(T->isPointerTy() || T->isArrayTy())),
                           raw.extract(0, 0) == ctx.bv_val(1, 1));
        };
        
        for (NodeRef ref : order) {
            Node& node = aeg.lookup(ref);
            
            z3::expr& taint = node.taint;
            
            switch (node.inst->kind()) {
                case Inst::Kind::ENTRY:
                case Inst::Kind::EXIT:
                    taint = ctx.bool_val(false);
                    break;
                    
                case Inst::Kind::LOAD: {
                    const z3::expr addr = node.get_memory_address();
                    const z3::expr addr_taint = get_value(ref, node.get_memory_address_pair().first);
                    taint = addr_taint || read(node.get_memory_address(), node.inst.I->getType());
                    break;
                }
                    
                case Inst::Kind::STORE: {
                    const llvm::StoreInst *S = llvm::cast<llvm::StoreInst>(node.inst.I);
                    taint = ctx.bool_val(false);
                    const z3::expr value = z3::ite(get_value(ref, S->getValueOperand()), ctx.bv_val(1, 2), ctx.bv_val(0, 2));
                    taint_mem = z3::conditional_store(taint_mem, node.get_memory_address(), value, node.exec()); // TODO: is exec() ok?
                    break;
                }
                    
                case Inst::Kind::OTHER: {
                    taint = ctx.bool_val(false);
                    if (llvm::isa<llvm::BranchInst>(node.inst.I)) { break; }
                    for (const llvm::Value *V : node.inst.I->operand_values()) {
                        taint = taint || get_value(ref, V);
                    }
                    break;
                }
                    
                default: std::abort();
                    
            }
        }
    }
#endif
}

z3::expr Taint_BV::get_value(NodeRef ref, const llvm::Value *V) const {
    const auto& node = aeg.po.lookup(ref);
    const auto it = node.refs.find(V);
    if (it == node.refs.end()) {
        if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
            return ctx.bool_val(true);
        } else if (const llvm::Constant *C = llvm::dyn_cast<llvm::Constant>(V)) {
            return ctx.bool_val(false);
        } else if (const llvm::BasicBlock *B = llvm::dyn_cast<llvm::BasicBlock>(V)) {
            return ctx.bool_val(false);
        } else {
            const auto I = std::get<const llvm::Instruction *>(node.v);
            if (llvm::isa<llvm::PHINode>(I)) {
                return ctx.bool_val(false);
            } else {
                llvm::errs() << "error: " << *V << "\n";
                std::abort();
            }
        }
    } else {
        // TODO: This might be a bug, or at least inaccurate.
        z3::expr taint = ctx.bool_val(false);
        for (const NodeRef src : it->second) {
            taint = taint || aeg.lookup(src).taint;
        }
        return taint;
    }
}
