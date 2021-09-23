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


inline bool to_bool(const z3::expr& e) {
    switch (e.bool_value()) {
        case Z3_L_TRUE: return true;
        case Z3_L_FALSE: return false;
        default: {
            std::stringstream ss;
            ss << "cannot concretize boolean " << e;
            throw z3::exception(ss.str().c_str());
        }
    }
}

struct eval {
    const z3::model& model;
    eval(const z3::model& model): model(model) {}
    
    bool get_bool(const z3::expr& e) const {
        return z3::to_bool(model.eval(e));
    }
};

}

namespace std {

template <>
struct hash<z3::expr> {
    auto operator()(const z3::expr& e) const {
        return e.id();
    }
};

}
