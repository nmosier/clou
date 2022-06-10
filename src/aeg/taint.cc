#include <llvm/IR/Operator.h>

#include "taint.h"
#include "aeg.h"
#include "cfg/expanded.h"
#include "passes/attacker-taint.h"

namespace aeg {

void AEG::construct_attacker_taint() {
    if (use_attacker_control_analysis) {
        for (NodeRef ref : node_range()) {
            Node& node = lookup(ref);
	    bool taint;
            if (const llvm::Instruction *I = node.inst->get_inst()) {
	      const auto it = attacker_taint.find(I->getFunction());
	      taint = (it == attacker_taint.end()) ? true : it->second.get(I);
            } else {
	      taint = false;
            }
	    node.attacker_taint.set(taint);
        }
    }
}

}
