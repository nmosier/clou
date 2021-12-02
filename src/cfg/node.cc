#include <iostream>

#include <llvm/Support/raw_ostream.h>

#include "cfg/node.h"
#include "util/functional.h"

namespace cfg {

std::ostream& operator<<(std::ostream& os, const Node::Call& call) {
    std::string s;
    llvm::raw_string_ostream ss {s};
    ss << *call.C << " " << *call.arg;
    return os << ss.str();
}

std::ostream& operator<<(std::ostream& os, const Node::Variant& v) {
    std::visit(util::overloaded {
        [&] (const auto *x) {
            std::string s;
            llvm::raw_string_ostream ss {s};
            ss << *x;
            os << ss.str();
        },
        [&] (const auto& x) {
            os << x;
        },
    }, v);
    return os;
}


}
