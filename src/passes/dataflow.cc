#include "dataflow.h"

namespace dataflow {

namespace impl {

void for_each_instruction(const llvm::Function& F, std::function<void (const llvm::Instruction *I)> func) {
    for (const llvm::BasicBlock& B : F) {
        for (const llvm::Instruction& I : B) {
            func(&I);
        }
    }
}

}


}
