#include <functional>

#include "z3.h"
#include "util.h" // for util::overloaded

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


void mysolver::push() {
    s.push();
    asserted_false_stack.push_back(asserted_false);
    asserted_false = false;
}

void mysolver::pop() {
    asserted_false = asserted_false_stack.back();
    asserted_false_stack.pop_back();
    s.pop();
}

void mysolver::check_expr(const z3::expr& e) {
    if (e.is_false()) {
        asserted_false = true;
    }
}

void mysolver::add(const z3::expr& e) {
    check_expr(e);
    s.add(e);
}

void mysolver::add(const z3::expr& e, const std::string& d) {
    check_expr(e);
    s.add(e, d.c_str());
}

void mysolver::add(const z3::expr_vector& v) {
    for (const z3::expr& e : v) {
        check_expr(e);
    }
    s.add(v);
}

bool mysolver::is_trivially_unsat() const {
    return asserted_false || std::any_of(asserted_false_stack.begin(), asserted_false_stack.end(), [] (bool x) { return x; });
}

z3::check_result mysolver::check() {
    if (is_trivially_unsat()) {
        return z3::unsat;
    } else {
        return s.check();
    }
}

z3::expr_vector mysolver::unsat_core() const {
    if (is_trivially_unsat()) {
        z3::expr_vector res {ctx()};
        res.push_back(ctx().bool_val(false));
        return res;
    } else {
        return s.unsat_core();
    }
}

}
