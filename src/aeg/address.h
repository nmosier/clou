#pragma once

#include <z3++.h>

#include "aeg/fwd.h"

namespace aeg {

#if 0

using Address = z3::expr;

#else

// TODO: inline this
struct Address {
    z3::expr arch;
    z3::expr trans;
    
    Address(Context& ctx);
    Address(Context& ctx, const z3::expr& arch);
};

std::ostream& operator<<(std::ostream& os, const Address& x);

#endif


}
