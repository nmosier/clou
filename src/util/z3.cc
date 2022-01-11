#include <functional>

#include "z3.h"
#include "util/hash.h"
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
    } else if (exprs.size() - 1 == count) {
        return !z3::mk_and(exprs);
    } else {
        return z3::atmost(exprs, count);
    }
}

z3::expr atleast2(const z3::expr_vector& exprs, unsigned count) {
    if (exprs.size() < count) {
        return exprs.ctx().bool_val(false);
    } else if (exprs.size() == count) {
        return z3::mk_and(exprs);
    } else if (count == 1) {
        return z3::mk_or(exprs);
    } else {
        return z3::atleast(exprs, count);
    }
}

std::unordered_map<std::pair<unsigned, unsigned>, unsigned> hist;
z3::expr exactly(const z3::expr_vector& exprs, unsigned count) {
#if 0
    hist[std::make_pair(exprs.size(), count)]++;
#endif
    
    z3::context& ctx = exprs.ctx();
    
    if (exprs.size() < count) {
        return ctx.bool_val(false);
    } else if (exprs.size() == 1 && count == 1) {
        return exprs.back();
    } else if (exprs.size() == 2 && count == 1) {
        return exprs[0] != exprs[1];
    } else if (exprs.size() == count) {
        return z3::mk_and(exprs);
    } else {
        const z3::expr lower = z3::atleast2(exprs, count);
        const z3::expr upper = z3::atmost2(exprs, count);
        return lower && upper;
    }
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

z3::expr translate(const z3::expr& e, z3::context& ctx) {
    Z3_ast e_ = Z3_translate(e.ctx(), e, ctx);
    return z3::expr(ctx, e_);
}

z3::expr_vector translate(const z3::expr_vector& v, z3::context& ctx) {
    Z3_ast_vector v_ = Z3_ast_vector_translate(v.ctx(), v, ctx);
    return z3::expr_vector(ctx, v_);
}

z3::solver translate(const z3::solver& s, z3::context& ctx) {
    if (Z3_solver_get_num_scopes(s.ctx(), s) != 0) {
        throw z3::exception("cannot translate scoped solver");
    }
    return z3::solver(ctx, s, z3::solver::translate());
}

z3::model translate(const z3::model& m, z3::context& ctx) {
    z3::model m_ = m;
    return z3::model(m_, ctx, z3::model::translate());
}

}
