#include <llvm/IR/Operator.h>

#include "taint.h"
#include "aeg.h"
#include "cfg/expanded.h"
#include "attacker-taint.h"

namespace aeg {

#if 0

void AEG::construct_attacker_taint() {
    z3::expr mem = z3::const_array(context->int_sort(), context->bool_val(true));
    
    for (NodeRef ref : po.reverse_postorder()) {
        // get taint of operands
        Node& node = lookup(ref);
        const auto& refs = po.lookup(ref).refs;
        z3::expr& value_taint = node.attacker_taint.value;
        value_taint = context->bool_val(false);
        
        const auto get_taint = [&] (const llvm::Value *V) {
            const auto it = refs.find(V);
            if (it == refs.end()) {
                if (llvm::isa<llvm::Argument>(V)) {
                    // is tainted
                    return context->bool_val(true);
                } else if (llvm::isa<llvm::GlobalValue, llvm::ConstantData, llvm::ConstantExpr, llvm::BlockAddress, llvm::MetadataAsValue, llvm::BasicBlock>(V)) {
                    // all constant pointers
                    return context->bool_val(false);
                } else if (llvm::isa<llvm::Operator>(V)) {
                    llvm::errs() << "stub: treating llvm::Operator as attacker-tainted: " << *V << "\n";
                    return context->bool_val(true);
                } else {
                    llvm::errs() << "unhandled src operand type " << *V << "\n";
                    std::abort();
                }
            } else {
                
                z3::expr_vector v {context};
                for (NodeRef src : it->second) {
                    v.push_back(lookup(src).attacker_taint.value);
                }
                return z3::mk_or(v);

            }
        };
        
        if (const llvm::Instruction *I = node.inst->get_inst()) {
            
            if (const llvm::LoadInst *LI = llvm::dyn_cast<llvm::LoadInst>(I)) {
                
                if (const llvm::GlobalVariable *GV = llvm::dyn_cast<llvm::GlobalVariable>(LI->getPointerOperand())) {
                    if (GV->isConstant()) {
                        value_taint = context->bool_val(false);
                        goto assigned;
                    }
                }
                
                {
                    const Address& addr = node.get_memory_address();
#if 0
                    const z3::expr addr_ = z3::ite(node.arch, addr.arch, addr.trans);
#else
                    const z3::expr addr_ = addr.arch;
#endif
                    value_taint = mem[addr_];
                }
                
            assigned: ;
                
                
            } else if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
                
                // get operand taint
                const StoreInst& SI_ = dynamic_cast<const StoreInst&>(*node.inst);
                const llvm::Value *V = SI_.get_value_operand();
                z3::expr value_taint = get_taint(V);
                
                const Address& addr = node.get_memory_address();
#if 0
                    const z3::expr addr_ = z3::ite(node.arch, addr.arch, addr.trans);
#else
                    const z3::expr addr_ = addr.arch;
#endif
                mem = z3::store(mem, addr_, value_taint);
                
            } else {
                
                for (const llvm::Value *V : I->operands()) {
                    value_taint = value_taint || get_taint(V);
                }
                
            }
        }
    }
}

#else

void AEG::construct_attacker_taint() {
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        
        if (const llvm::Instruction *I = node.inst->get_inst()) {
            node.attacker_taint.value = context->bool_val(attacker_taint.get(I));
        } else {
            node.attacker_taint.value = context->bool_val(false);
        }
    }
}

#endif

}
