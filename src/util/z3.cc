#include <functional>

#include "z3.h"
#include "util/functional.h"

namespace z3 {

z3::expr max(const z3::expr_vector& v) {
    if (v.empty()) {
        throw z3::exception("max taken of empty vector");
    }
    
    auto it = v.begin();
    const auto& init = *it++;
    return lfold(it, v.end(), init, [] (const z3::expr& a, const z3::expr& b) -> z3::expr {
        return z3::max(a, b);
    });
}

z3::expr min(const z3::expr_vector& v) {
    if (v.empty()) {
        throw z3::exception("min taken of empty vector");
    }
    
    auto it = v.begin();
    const auto& init = *it++;
    return lfold(it, v.end(), init, [] (const z3::expr& a, const z3::expr& b) -> z3::expr {
        return z3::min(a, b);
    });
}

z3::expr atmost2(const z3::expr_vector& exprs, unsigned count) {
    if (exprs.size() <= count) {
        return exprs.ctx().bool_val(true);
    } else {
        return z3::atmost(exprs, count);
    }
}

z3::expr atleast2(const z3::expr_vector& exprs, unsigned count) {
    if (exprs.size() < count) {
        return exprs.ctx().bool_val(false);
    } else if (exprs.size() == count) {
        return z3::mk_and(exprs);
    } else {
        return z3::atleast(exprs, count);
    }
}

z3::expr exactly(const z3::expr_vector& exprs, unsigned count) {
    const z3::expr lower = z3::atleast2(exprs, count);
    const z3::expr upper = z3::atmost2(exprs, count);
    return lower && upper;
}

z3::solver duplicate(const z3::solver& orig) {
    z3::context& ctx = orig.ctx();
    z3::solver dup {ctx};
    for (const z3::expr& e : orig.assertions()) {
        dup.add(e);
    }
    return dup;
}

z3::expr distinct2(const z3::expr_vector& v) {
    if (v.empty()) {
        return v.ctx().bool_val(true);
    } else {
        return distinct(v);
    }
}

}
