#pragma once

#include <string>
#include <ostream>
#include <optional>

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

bool getelementptr_can_zero(const llvm::GetElementPtrInst *GEP);
std::optional<int> getelementptr_const_offset(const llvm::GetElementPtrInst *GEP);

bool contains_struct(const llvm::Type *T);

bool pointer_is_read_only(const llvm::Value *P);

}

template <typename T>
std::ostream& llvm_to_cxx_os(std::ostream& os, const T& x) {
    return os << llvm::to_string(x);
}

inline std::ostream& operator<<(std::ostream& os, const llvm::Instruction& I) {
   return llvm_to_cxx_os(os, I);
}

