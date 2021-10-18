#pragma once

#include <string>
#include <ostream>

#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Instruction.h>

namespace llvm {
template <typename T>
std::string to_string(const T& x) {
    std::string s;
    llvm::raw_string_ostream ss {s};
    ss << x;
    return s;
}
}

template <typename T>
std::ostream& llvm_to_cxx_os(std::ostream& os, const T& x) {
    return os << llvm::to_string(x);
}

inline std::ostream& operator<<(std::ostream& os, const llvm::Instruction& I) {
   return llvm_to_cxx_os(os, I);
}


#if 0
namespace detail {

template <typename InputIt>

template <typename Container>
llvm::raw_ostream& print_container_impl(llvm::raw_ostream& os, const Container& container)

}


template <typename T>
llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const std::unordered_set<T>& set) {
    os << "{";
    for (auto it = set.begin(); it != set.end(); ++it) {
        if (it != set.begin()) {
            os << ", ";
        }
        os << *it;
    }
    os << "}";
    return os;
}

template <typename T>
llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const std::vector<T>& vec) {
    os << "{";
    for (auto it = vec.begin(); )
}
#endif
