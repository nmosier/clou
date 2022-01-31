#pragma once

#include <z3++.h>

namespace aeg {

struct Taint {
    z3::expr value;
    // z3::expr memory;
    
    Taint(z3::context& ctx): value(ctx) {}
};

}
