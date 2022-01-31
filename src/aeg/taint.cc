#include <llvm/IR/Operator.h>

#include "taint.h"
#include "aeg.h"
#include "cfg/expanded.h"

namespace aeg {

void AEG::construct_attacker_taint() {
    z3::expr mem = z3::const_array(context->int_sort(), context->bool_val(true));
    
    for (NodeRef ref : po.reverse_postorder()) {
        // get taint of operands
        Node& node = lookup(ref);
        const auto& refs = po.lookup(ref).refs;
        z3::expr& value_taint = node.attacker_taint.value;
        value_taint = context->bool_val(false);
        
        const auto get_taint = [&] (const llvm::Value *V, z3::expr& value_taint) {
            const auto it = refs.find(V);
            if (it == refs.end()) {
                if (llvm::isa<llvm::Argument>(V)) {
                    // is tainted
                    value_taint = context->bool_val(true);
                } else if (llvm::isa<llvm::GlobalValue, llvm::ConstantData, llvm::ConstantExpr, llvm::BlockAddress, llvm::MetadataAsValue, llvm::BasicBlock>(V)) {
                    // all constant pointers
                    value_taint = context->bool_val(false);
                } else if (llvm::isa<llvm::Operator>(V)) {
                    llvm::errs() << "stub: treating llvm::Operator as attacker-tainted: " << *V << "\n";
                    value_taint = context->bool_val(true);
                } else {
                    llvm::errs() << "unhandled src operand type " << *V << "\n";
                    std::abort();
                }
            } else {
                
                for (NodeRef src : it->second) {
                    value_taint = value_taint || lookup(src).attacker_taint.value;
                }

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
                    const z3::expr addr_ = z3::ite(node.arch, addr.arch, addr.trans);
                    value_taint = mem[addr_];
                }
                
            assigned: ;
                
                
            } else if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
                
                // get operand taint
                const StoreInst& SI_ = dynamic_cast<const StoreInst&>(*node.inst);
                const llvm::Value *V = SI_.get_value_operand();
                z3::expr value_taint = context->bool_val(false);
                get_taint(V, value_taint);
                
                const Address& addr = node.get_memory_address();
                const z3::expr addr_ = z3::ite(node.arch, addr.arch, addr.trans);
                mem = z3::store(mem, addr_, value_taint);
                
            } else {
                
                for (const llvm::Value *V : I->operands()) {
                    get_taint(V, value_taint);
                }
                
            }
        }
    }
}

}
