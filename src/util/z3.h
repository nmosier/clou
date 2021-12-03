#pragma once

#include <type_traits>
#include <unordered_map>
#include <vector>
#include <string>
#include <variant>
#include <optional>
#include <algorithm>
#include <array>
#include <numeric>
#include <iterator>

#include <z3++.h>

#include "util/output.h"


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
z3::expr min(const z3::expr_vector& v);
z3::expr atmost2(const z3::expr_vector& exprs, unsigned count);
z3::expr atleast2(const z3::expr_vector& exprs, unsigned count);
z3::expr exactly(const z3::expr_vector& exprs, unsigned count);

// DEBUG
extern std::unordered_map<std::pair<unsigned, unsigned>, unsigned> hist;

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

#if 1
template <typename Solver>
class scope {
    static_assert(std::is_class<Solver>());
public:
    scope(Solver& solver) { open(&solver); }
    scope(Solver& solver, const std::string& pop_msg): pop_msg(pop_msg) { open(&solver); }
    scope(const scope& other) = delete;
    scope(scope&& other) {
        if (good()) { close(); }
        solver = other.solver; other.solver = nullptr;
        pop_msg = std::move(other.pop_msg);
    }
    scope& operator=(const scope& other) = delete;
    
    
    ~scope() {
        if (good()) { close(); }
    }
    
    z3::context& ctx() const { return solver->ctx(); }
    
    bool good() const { return solver != nullptr; }
    operator bool() const { return good(); }
    
private:
    Solver *solver = nullptr;
    std::optional<std::string> pop_msg;
    
    void open(Solver *solver) {
        assert(!good());
        this->solver = solver;
        this->solver->push();
    }
    
    void close() {
        assert(good());
        solver->pop();
        solver = nullptr;
        if (pop_msg) {
            std::cerr << *pop_msg;
        }
    }
};
#else
template <class Solver>
class scope {
    static_assert(std::is_class<Solver>());
public:
    scope(Solver& solver): solver(solver) { solver.push(); }
    ~scope() { solver.pop(); }
    z3::context& ctx() const { return solver.ctx(); }
private:
    Solver& solver;
};
#endif
#define z3_scope const z3::scope<std::remove_reference<decltype(solver)>::type> scope {solver}


/** Enumerate all the possible values of the given expression \p expr under the current constraints in \p solver.
 * \param solver The solver to use
 * \param expr The expression to enumerate possible values of
 * \param out Output iterator accepting the possible values with type z3::expr
 */
template <typename OutputIt>
OutputIt enumerate(z3::solver& solver, const z3::expr& expr, OutputIt out) {
    std::cerr << "TODO: " << __FUNCTION__ << "\n";
    std::abort();
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

/** This solver class catches assertions that make the query trivially unsatisfiable. Currently, it checks if an trivial_solver::add()'ed instruction is z3::expr::is_false(). */
template <class Solver>
class trivial_solver {
    static_assert(std::is_class<Solver>());
public:
    using solver_type = Solver;
    explicit trivial_solver(z3::context& c): s(c) {}
    explicit trivial_solver(const z3::solver& s): s(s) {}
    z3::context& ctx() const { return s.ctx(); }
    void push();
    void pop();
    void add(const z3::expr& e);
    void add(const z3::expr& e, const std::string& d);
    void add(const z3::expr_vector& v);
    z3::check_result check();
    z3::check_result check(const z3::expr_vector& assumptions);
    z3::model get_model() const { return s.get_model(); }
    z3::stats statistics() const { return s.statistics(); }
    z3::expr_vector unsat_core() const;
    z3::expr_vector assertions() const { return s.assertions(); }

    operator const z3::solver&() const { return s; }
    operator z3::solver&() { return s; }
    
private:
    Solver s;
    bool asserted_false = false;
    std::vector<bool> asserted_false_stack;
    
    void check_expr(const z3::expr& e);
    bool is_trivially_unsat() const;
};

template <class Solver>
void trivial_solver<Solver>::push() {
    s.push();
    asserted_false_stack.push_back(asserted_false);
    asserted_false = false;
}

template <class Solver>
void trivial_solver<Solver>::pop() {
    asserted_false = asserted_false_stack.back();
    asserted_false_stack.pop_back();
    s.pop();
}

template <class Solver>
void trivial_solver<Solver>::check_expr(const z3::expr& e) {
    if (e.is_false()) {
        asserted_false = true;
    }
}

template <class Solver>
void trivial_solver<Solver>::add(const z3::expr& e) {
    check_expr(e);
    s.add(e);
}

template <class Solver>
void trivial_solver<Solver>::add(const z3::expr& e, const std::string& d) {
    check_expr(e);
    s.add(e, d.c_str());
}

template <class Solver>
void trivial_solver<Solver>::add(const z3::expr_vector& v) {
    for (const z3::expr& e : v) {
        check_expr(e);
    }
    for (const z3::expr& e : v) {
        s.add(e);
    }
}

template <class Solver>
bool trivial_solver<Solver>::is_trivially_unsat() const {
    return asserted_false || std::any_of(asserted_false_stack.begin(), asserted_false_stack.end(), [] (bool x) { return x; });
}

template <class Solver>
z3::check_result trivial_solver<Solver>::check() {
    return check(z3::expr_vector(ctx()));
}

template <class Solver>
z3::check_result trivial_solver<Solver>::check(const z3::expr_vector& assumptions) {
    if (is_trivially_unsat()) {
        return z3::unsat;
    }
    for (const z3::expr& assumption : assumptions) {
        if (assumption.is_false()) {
            return z3::unsat;
        }
    }
    return s.check();
}

template <class Solver>
z3::expr_vector trivial_solver<Solver>::unsat_core() const {
    if (is_trivially_unsat()) {
        z3::expr_vector res {ctx()};
        res.push_back(ctx().bool_val(false));
        return res;
    } else {
        return s.unsat_core();
    }
}




template <class Solver>
class lazy_solver {
public:
    using solver_type = Solver;
    explicit lazy_solver(z3::context& c): s(c) {}
    explicit lazy_solver(const z3::solver& s): s(s) {}
    lazy_solver(const lazy_solver&) = delete;
    lazy_solver& operator=(const lazy_solver&) = delete;
    z3::context& ctx() const { return s.ctx(); }
    void push();
    void pop();
    void add(const z3::expr& e) { cur.emplace_back(e); }
    void add(const z3::expr& e, const std::string& d) { cur.emplace_back(e, d); }
    void add(const z3::expr_vector& v);
    z3::check_result check() { return check(z3::expr_vector(ctx())); }
    z3::check_result check(const z3::expr_vector& assumptions);
    z3::model get_model() const { return s.get_model(); }
    z3::expr_vector unsat_core() const { return s.unsat_core(); }
    z3::expr_vector assertions() const;
    
private:
    Solver s;
    struct Entry {
        z3::expr e;
        std::optional<std::string> d;
        
        Entry(const z3::expr& e): e(e) {}
        Entry(const z3::expr& e, const std::string& d): e(e), d(d) {}
    };
    using Uncommitted = std::vector<Entry>;
    Uncommitted cur;
    std::vector<Uncommitted> stack;
    
    void commit(const Entry& entry);
    void commit(const Uncommitted& v);
};

/*
 * On add(), place into `cur`.
 * Clear the scopes whenever we query.
 */

template <class Solver>
void lazy_solver<Solver>::push() {
    stack.push_back(std::move(cur));
}

template <class Solver>
void lazy_solver<Solver>::pop() {
    if (stack.empty()) {
        cur.clear();
        s.pop();
    } else {
        cur = std::move(stack.back());
        stack.pop_back();
    }
}

template <class Solver>
void lazy_solver<Solver>::add(const z3::expr_vector& v) {
    for (const z3::expr& e : v) {
        add(e);
    }
}

template <class Solver>
void lazy_solver<Solver>::commit(const Entry& entry) {
    if (entry.d) {
        s.add(entry.e, entry.d->c_str());
    } else {
        s.add(entry.e);
    }
}

template <class Solver>
void lazy_solver<Solver>::commit(const Uncommitted& v) {
    for (const Entry& ent : v) {
        commit(ent);
    }
}

template <class Solver>
z3::check_result lazy_solver<Solver>::check(const z3::expr_vector& assumptions) {
    /* commit all scopes so far */
    for (const Uncommitted& v : stack) {
        commit(v);
        s.push();
    }
    stack.clear();
    
    /* commit current assertions */
    commit(cur);
    cur.clear();
    
    /* check using base solver */
    if (assumptions.empty()) {
        return s.check();
    } else {
        return s.check(assumptions);
    }
}

template <class Solver>
z3::expr_vector lazy_solver<Solver>::assertions() const {
    z3::expr_vector res = s.assertions();
    const auto f = [&res] (const Uncommitted& v) {
        for (const Entry& ent : v) {
            res.push_back(ent.e);
        }
    };
    for (const Uncommitted& scope : stack) {
        f(scope);
    }
    f(cur);
    return res;
}


template <typename Solver>
class simplify_solver {
    static_assert(std::is_class<Solver>());
public:
    explicit simplify_solver(z3::context& ctx): s(ctx) {}
    explicit simplify_solver(const z3::solver& solver): s(solver) {}
    simplify_solver(const simplify_solver&) = delete;
    simplify_solver& operator=(const simplify_solver&) = delete;
    
    z3::context ctx() const { return s.ctx(); }
    void push() { s.push(); }
    void pop() { s.pop(); }
    void add(const z3::expr& e) { s.add(e.simplify()); }
    void add(const z3::expr& e, const std::string& d) { s.add(e.simplify(), d.c_str()); }
    void add(const z3::expr_vector& v);
    z3::check_result check() { return s.check(); }
    z3::model get_model() const { return s.get_model(); }
    z3::expr_vector unsat_core() const { return s.unsat_core(); }
    z3::expr_vector assertions() const { return s.assertions(); }
    
private:
    Solver s;
};

template <typename Solver>
void simplify_solver<Solver>::add(const z3::expr_vector& v) {
    const z3::expr_vector v_ = z3::transform(ctx(), v, [] (const z3::expr& e) -> z3::expr {
        return e.simplify();
    });
    s.add(v_);
}


template <class Solver>
class cache_solver {
    static_assert(std::is_class<Solver>());
public:
    cache_solver(z3::context& ctx): s(ctx) {}
    cache_solver(z3::solver& solver): s(solver) {}
    cache_solver(const cache_solver&) = delete;
    cache_solver& operator=(const cache_solver&) = delete;
    
    z3::context& ctx() const { return s.ctx(); }
    void push();
    void pop();
    void add(const z3::expr& e) { add_impl(e); }
    void add(const z3::expr& e, const std::string& d) { add_impl(e, d.c_str()); }
    void add(const z3::expr_vector& v);
    z3::check_result check();
    z3::model get_model() const { return s.get_model(); }
    z3::expr_vector unsat_core() const { return s.unsat_core(); }
    z3::expr_vector assertions() const { return s.assertions(); }
    
private:
    Solver s;
    using Result = std::optional<z3::check_result>;
    Result cur;
    std::vector<Result> stack;
    
    template <typename... Args>
    void add_impl(Args&&... args) {
        cur = std::nullopt;
        s.add(std::forward<Args>(args)...);
    }
};

template <class Solver>
void cache_solver<Solver>::push() {
    stack.push_back(cur);
    s.push();
}

template <class Solver>
void cache_solver<Solver>::pop() {
    s.pop();
    cur = stack.back();
    stack.pop_back();
}

template <class Solver>
void cache_solver<Solver>::add(const z3::expr_vector& v) {
    if (!v.empty()) {
        add_impl(v);
    }
}

template <class Solver>
z3::check_result cache_solver<Solver>::check() {
    if (!cur) {
        cur = s.check();
    } else {
        std::cerr << "mark:cached-check\n";
    }
    return *cur;
}

namespace detail {

/* STEP 1: map vectors to their corresponding (variable, equality-assertion) pairs
 * STEP 2:
 */

}

template <typename InputIt>
z3::expr no_intersect(z3::context& ctx, const z3::sort& sort, const char *name, InputIt begin, InputIt end) {
    const z3::func_decl func = ctx.function(name, 1, &sort, ctx.int_sort());
    
    z3::expr_vector assertions {ctx};
    int i = 0;
    for (auto it = begin; it != end; ++it, ++i) {
        for (const z3::expr& e : *it) {
            assertions.push_back(func(e) == i);
        }
    }
    
    return z3::mk_and(assertions);
}

template <typename Container>
z3::expr no_intersect(z3::context& ctx, const z3::sort& sort, const char *name, const Container& container) {
    return no_intersect(ctx, sort, name, container.begin(), container.end());
}

inline z3::expr no_intersect(const char *name, const z3::sort& sort, const z3::expr_vector& set1, const z3::expr_vector& set2) {
    const std::array<z3::expr_vector, 2> sets = {set1, set2};
    return no_intersect(set1.ctx(), sort, name, sets);
}

template <class InputIt, class BinaryOp>
z3::expr lfold(InputIt begin, InputIt end, const z3::expr& init, BinaryOp op) {
    z3::expr acc = init;
    for (auto it = begin; it != end; ++it) {
        acc = op(acc, *it);
    }
    return acc;
}

template <class Solver>
z3::check_result check_timeout(Solver& solver_, unsigned nsecs) {
    auto& solver = static_cast<z3::solver&>(solver_);
    solver.set("timeout", nsecs);
    const auto res = solver.check();
    solver.set("timeout", 0U);
    return res;
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

