#pragma once

#include <vector>

#include <llvm/Analysis/AliasAnalysis.h>

namespace aeg {
class AliasAnalysis;
}

#include "aeg/aeg.h"

namespace aeg {

class AliasAnalysis {
public:
    AliasAnalysis(AEG& aeg, llvm::AliasAnalysis& AA): aeg(aeg), AA(AA) {}
    
private:
    using AddrInfoVec = std::vector<AEG::AddrInfo>;
    aeg::AEG& aeg;
    llvm::AliasAnalysis& AA;
    
    void init();
    
};

}
