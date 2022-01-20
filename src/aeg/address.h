#pragma once

#include <z3++.h>

#include "aeg/fwd.h"

namespace aeg {


using Address = z3::expr;

#if 0
// TODO: inline this
struct Address {
    z3::expr addr;
    
    operator const z3::expr& () const { return addr; }

    Address(Context& ctx);
    Address(const z3::expr& addr): addr(addr) {}
    
    z3::expr operator==(const Address& other) const {
        return addr == other.addr;
    }
};

inline std::ostream& operator<<(std::ostream& os, const Address& x) {
    return os << "addr(" << x.addr << ")";
}
#endif


}
