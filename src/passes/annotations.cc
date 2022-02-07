#include <unordered_map>
#include <string>

#include "util/llvm.h"
#include "annotations.h"

bool AnnotationPass::runOnModule(llvm::Module& M) {
    std::unordered_map<const llvm::Value *, std::string> map;
    llvm::parse_annotations(M, std::inserter(map, map.end()));
    
    for (const auto& p : map) {
        llvm::errs() << *p.first << " - " << p.second << "\n";
    }
    
    return false;
}

namespace {

llvm::RegisterPass<AnnotationPass> X {
    "print_annotations", "Print Annotations Pass"
};

}
