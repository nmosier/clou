#pragma once

#include <type_traits>
#include <unordered_map>

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

inline z3::expr atmost2(const z3::expr_vector& exprs, unsigned count) {
    if (exprs.size() <= count) {
        return exprs.ctx().bool_val(true);
    } else {
        return z3::atmost(exprs, count);
    }
}

inline z3::expr atleast2(const z3::expr_vector& exprs, unsigned count) {
    if (exprs.size() < count) {
        return exprs.ctx().bool_val(false);
    } else {
        return z3::atleast(exprs, count);
    }
}

inline z3::expr exactly(const z3::expr_vector& exprs, unsigned count) {
    const z3::expr lower = z3::atleast2(exprs, count);
    const z3::expr upper = z3::atmost2(exprs, count);
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

struct concrete_value {
    z3::expr e;
    explicit operator bool() const { return z3::to_bool(e); }
    explicit operator unsigned() const { return e.get_numeral_uint(); }
};

inline z3::expr operator==(const concrete_value& a, const z3::expr& b) {
    return a.e == b;
}

inline z3::expr operator==(const z3::expr& a, const concrete_value& b) {
    return a == b.e;
}

struct eval {
    const z3::model model;
    explicit eval(const z3::model& model): model(model) {}
    concrete_value operator()(const z3::expr& val) const {
        return {model.eval(val, true)};
    }
    z3::expr equal(const z3::expr& e) const {
        return e == model.eval(e, true);
    }
};

inline z3::expr conditional_store(const z3::expr& a, const z3::expr& i, const z3::expr& v, const z3::expr& c) {
    return store(a, i, ite(c, v, a[i]));
}

struct scope {
    z3::solver& solver;
    scope(z3::solver& solver): solver(solver) {
        solver.push();
    }
    ~scope() {
        solver.pop();
    }
};
#define z3_scope const z3::scope scope {solver}


/** Enumerate all the possible values of the given expression \p expr under the current constraints in \p solver.
 * \param solver The solver to use
 * \param expr The expression to enumerate possible values of
 * \param out Output iterator accepting the possible values with type z3::expr
 */
template <typename OutputIt>
OutputIt enumerate(z3::solver& solver, const z3::expr& expr, OutputIt out) {
    z3_scope;
    while (solver.check() == z3::sat) {
        const z3::eval eval {solver.get_model()};
        *out++ = eval(expr).e;
        solver.add(expr != eval(expr).e);
    }
    return out;
}

inline std::ostream& operator<<(std::ostream& os, const z3::concrete_value& x) {
    return os << x.e;
}

inline bool always_true(z3::solver& solver, const z3::expr& pred) {
    z3::expr_vector vec {solver.ctx()};
    vec.push_back(!pred);
    return solver.check(vec) == z3::unsat;
}

inline z3::solver duplicate(const z3::solver& orig) {
    z3::context& ctx = orig.ctx();
    z3::solver dup {ctx};
    dup.add(orig.assertions());
    return dup;
}

template <typename... Args>
inline z3::check_result check_force(z3::solver& solver, Args&&... args) {
    const auto res = solver.check(std::forward<Args>(args)...);
    switch (res) {
        case z3::sat:
        case z3::unsat:
            return res;
        case z3::unknown:
            throw z3::exception("check result unknown");
    }
}

}

namespace std {

template <>
struct hash<z3::expr> {
    auto operator()(const z3::expr& e) const {
        return e.id();
    }
};

}
