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

#if 0
template <typename Functor>
struct evaluator {
    const z3::model& model;
    Functor func;
    evaluator(const z3::model& model, Functor functor): model(model), func(func) {}
    
    template <typename... Args>
    bool operator()(Args&&... args) const {
        const z3::expr res = func(std::forward<Args>(args)...);
        switch (model.eval(res).bool_value()) {
            case z3::Z3_L_FALSE: return false;
            case z3::Z3_L_TRUE: return true;
            default: std::abort();
        }
    }
};
#endif

}

namespace std {

template <>
struct hash<z3::expr> {
    auto operator()(const z3::expr& e) const {
        return e.id();
    }
};

}
