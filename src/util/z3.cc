#include <functional>

#include "z3.h"
#include "util/functional.h"

namespace z3 {

z3::expr max(const z3::expr_vector& exprs) {
    assert(!exprs.empty());
    auto it = exprs.begin();
    z3::expr acc = *it++;
    while (it != exprs.end()) {
        acc = min(acc, *it++);
    }
    return acc;
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
