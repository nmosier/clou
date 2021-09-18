#pragma once

#include <type_traits>
#include <z3++.h>

inline z3::expr& operator&=(z3::expr& a, const z3::expr& b) {
   a = a && b;
   return a;
}

inline z3::expr& operator|=(z3::expr& a, const z3::expr& b) {
   a = a || b;
   return a;
}


namespace z3 {
inline z3::expr max(const z3::expr_vector& exprs) {
    assert(!exprs.empty());
    auto it = exprs.begin();
    z3::expr acc = *it++;
    while (it != exprs.end()) {
        acc = min(acc, *it++);
    }
    return acc;
}

inline z3::expr exactly(const z3::expr_vector& exprs, unsigned count) {
    const z3::expr lower = z3::atleast(exprs, count);
    const z3::expr upper = z3::atmost(exprs, count);
    return lower && upper;
}
}
