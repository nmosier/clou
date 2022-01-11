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
// TODO: why is this necessary?
class scope {
public:
    scope(z3::solver& solver) { open(&solver); }
    scope(z3::solver& solver, const std::string& pop_msg): pop_msg(pop_msg) { open(&solver); }
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
    z3::solver *solver = nullptr;
    std::optional<std::string> pop_msg;
    
    void open(z3::solver *solver) {
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
#define z3_scope const z3::scope scope {solver}


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


z3::expr translate(const z3::expr& e, z3::context& ctx);
z3::expr_vector translate(const z3::expr_vector& v, z3::context& ctx);
z3::solver translate(const z3::solver& solver, z3::context& ctx);
z3::model translate(const z3::model& model, z3::context& ctx);

}

namespace std {

template <>
struct hash<z3::expr> {
    auto operator()(const z3::expr& e) const {
        return e.id();
    }
};

}

