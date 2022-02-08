#include <unordered_map>
#include <string>

#include "util/llvm.h"
#include "annotations.h"

bool AnnotationPass::runOnModule(llvm::Module& M) {
    llvm::parse_annotations(M, std::inserter(results, results.end()));
    return false;
}

void AnnotationPass::print(llvm::raw_ostream& os, const llvm::Module *M) const {
    for (const auto& p : results) {
        os << *p.first << "  -  " << p.second << "\n";
    }
}

namespace {

llvm::RegisterPass<AnnotationPass> X {
    "annotations", "Annotations Pass"
};

}
