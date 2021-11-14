#include "aeg/constraints.h"
#include "util/z3.h"

namespace aeg {

z3::expr Constraints::get(z3::context& ctx) const {
    return z3::mk_and(z3::transform(ctx, exprs, [] (const auto& p) {
        return p.first;
    }));
}


void Constraints::operator()(const z3::expr& clause, const std::string& name) {
    assert(!name.empty());
    if ((simplify_before_checking_for_false_constraints ? clause.simplify() : clause).is_false()) {
        std::cerr << "adding false constraint: " << clause << "\n";
        throw std::logic_error("adding constraint 'false'");
    }
    exprs.emplace_back(clause, name);
}


void Constraints::simplify() {
    std::for_each(exprs.begin(), exprs.end(), [] (auto& p) {
        z3::expr& e = p.first;
        e = e.simplify();
    });
}





}
