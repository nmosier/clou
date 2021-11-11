#pragma once

#include <type_traits>
#include <unordered_map>
#include <vector>
#include <string>
#include <variant>
#include <optional>

#include <z3++.h>


// TODO: remove
inline z3::expr& operator&=(z3::expr& a, const z3::expr& b) {
   a = a && b;
   return a;
}

inline z3::expr& operator|=(z3::expr& a, const z3::expr& b) {
   a = a || b;
   return a;
}


namespace z3 {

z3::expr max(const z3::expr_vector& exprs);
z3::expr atmost2(const z3::expr_vector& exprs, unsigned count);
z3::expr atleast2(const z3::expr_vector& exprs, unsigned count);
z3::expr exactly(const z3::expr_vector& exprs, unsigned count);

/// NOTE: bounds are inclusive
// TODO: delete?
inline z3::expr atleastmost(const z3::expr_vector& exprs, unsigned lower, unsigned upper) {
    return z3::atleast2(exprs, lower) && z3::atmost2(exprs, upper);
}

// TODO: delete?
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
    z3::context& ctx() const {
        return model.ctx();
    }
};
#define z3_eval const z3::eval eval {solver.get_model()}

inline z3::expr conditional_store(const z3::expr& a, const z3::expr& i, const z3::expr& v, const z3::expr& c) {
    return store(a, i, ite(c, v, a[i]));
}

template <typename Solver>
struct scope {
    Solver solver;
    scope(const Solver& solver): solver(solver) {
        this->solver.push();
    }
    ~scope() {
        solver.pop();
    }
};
#define z3_scope const z3::scope<decltype(solver)> scope {solver}


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

z3::solver duplicate(const z3::solver& orig);

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

class checker {
public:
    checker(z3::solver& solver): solver(solver) {}
    
    z3::check_result operator()() {
        if (!result) {
            result = solver.check();
        }
        return *result;
    }
    
private:
    z3::solver& solver;
    std::optional<z3::check_result> result;
};


template <typename InputIt, typename Op>
z3::expr_vector transform(z3::context& ctx, InputIt begin, InputIt end, Op op) {
    z3::expr_vector vec {ctx};
    for (auto it = begin; it != end; ++it) {
        vec.push_back(op(*it));
    }
    return vec;
}

template <typename InputIt, typename Op>
z3::expr_vector transform(InputIt begin, InputIt end, Op op) {
    assert(begin != end);
    z3::context& ctx = op(*begin).ctx();
    return transform(ctx, begin, end, op);
}

template <typename Container, typename Op>
z3::expr_vector transform(z3::context& ctx, const Container& container, Op op) {
    return transform(ctx, container.begin(), container.end(), op);
}

template <typename Container, typename Op>
z3::expr_vector transform(const Container& container, Op op) {
    return transform(container.begin(), container.end(), op);
}

z3::expr distinct2(const z3::expr_vector& v);

class mysolver {
public:
    mysolver(z3::context& c): s(c) {}
    mysolver(z3::solver& s): s(s) {}
    z3::context& ctx() const { return s.ctx(); }
    void push();
    void pop();
    void add(const z3::expr& e);
    void add(const z3::expr& e, const std::string& d);
    void add(const z3::expr_vector& v);
    z3::check_result check();
    z3::model get_model() const { return s.get_model(); }
    z3::stats statistics() const { return s.statistics(); }
    z3::expr_vector unsat_core() const;
    z3::expr_vector assertions() const { return s.assertions(); }

    operator const z3::solver&() const { return s; }
    operator z3::solver&() { return s; }
    
private:
    z3::solver s;
    bool asserted_false = false;
    std::vector<bool> asserted_false_stack;
    
    void check_expr(const z3::expr& e);
    bool is_trivially_unsat() const;
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

