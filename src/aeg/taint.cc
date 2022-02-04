#include <llvm/IR/Operator.h>

#include "taint.h"
#include "aeg.h"
#include "cfg/expanded.h"
#include "passes/attacker-taint.h"

namespace aeg {

void AEG::construct_attacker_taint() {
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        
        if (const llvm::Instruction *I = node.inst->get_inst()) {
            node.attacker_taint.set(attacker_taint.at(I->getFunction()).get(I));
        } else {
            node.attacker_taint.set(false);
        }
    }
}

}
