#pragma once

#include <z3++.h>

#include "aeg/fwd.h"

namespace aeg {


// TODO: inline this
struct Address {
    z3::expr addr;
    
    Address(Context& ctx);
    Address(Context& ctx, const z3::expr& addr);
    
    operator const z3::expr&() const {
        return addr;
    }
};

std::ostream& operator<<(std::ostream& os, const Address& x);

}
