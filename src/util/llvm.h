#pragma once

#include <string>
#include <ostream>

#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Instructions.h>

namespace llvm {
template <typename T>
std::string to_string(const T& x) {
    std::string s;
    llvm::raw_string_ostream ss {s};
    ss << x;
    return s;
}

#if 1
inline bool getelementptr_can_zero(const llvm::GetElementPtrInst *GEP) {
    for (const llvm::Value *V : GEP->indices()) {
        if (const llvm::ConstantInt *CI = llvm::dyn_cast<llvm::ConstantInt>(V)) {
            if (CI->getValue().getLimitedValue() != 0) {
                return false;
            }
        }
    }
    return true;
}
#endif
}

template <typename T>
std::ostream& llvm_to_cxx_os(std::ostream& os, const T& x) {
    return os << llvm::to_string(x);
}

inline std::ostream& operator<<(std::ostream& os, const llvm::Instruction& I) {
   return llvm_to_cxx_os(os, I);
}


