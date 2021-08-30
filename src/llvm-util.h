#pragma once

#include <string>
#include <ostream>

#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Instruction.h>

template <typename T>
std::ostream& llvm_to_cxx_os(std::ostream& os, const T& x) {
   std::string s;
   llvm::raw_string_ostream ss {s};
   ss << x;
   os << s;
   return os;
}

inline std::ostream& operator<<(std::ostream& os, const llvm::Instruction& I) {
   return llvm_to_cxx_os(os, I);
}
