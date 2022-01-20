#include "aeg/constraints.h"
#include "util/z3.h"
#include "config.h"
#include "util/output.h"
#include "aeg.h"
#include "cfg/expanded.h"

namespace aeg {

z3::expr Constraints::get(z3::context& ctx) const {
    return z3::mk_and(z3::transform(ctx, exprs, [] (const auto& p) {
        return p.first;
    }));
}

void Constraints::add_to(std::function<void (const z3::expr&, const std::string&)> func) const {
    for (const auto& expr : exprs) {
        func(expr.first, expr.second);
    }
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


void Constraints::dump(std::unordered_map<std::string, unsigned>& hist) const {
    for (const auto& p : exprs) {
        const std::string key = p.second.substr(0, p.second.find(':'));
        hist[key] += util::to_string(p.first).size();
    }
}



void AEG::constrain_arch() {
    // one exit arch
    constraints(z3::exactly(z3::transform(exits, [&] (NodeRef ref) -> z3::expr {
        return lookup(ref).arch;
    }), 1), "one-exit-arch");
}

void AEG::constrain_arch(const NodeRefSet& window, AssertFunc solver_add) {
    z3::expr_vector v {context};
    for (NodeRef exit : exits) {
        if (window.contains(exit)) {
            v.push_back(lookup(exit).arch);
        }
    }
    
    solver_add(z3::exactly(v, 1), "one-exit-arch");
}

void AEG::constrain_exec() {
    // exclusive architectural/transient execution
    for (const NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        std::stringstream ss;
        ss << "excl-exec:" << ref;
        node.constraints(!(node.arch && node.trans), ss.str());
    }
    
    constrain_trans();
}

void AEG::constrain_exec(const NodeRefSet& window, AssertFunc solver_add) {
    for (NodeRef ref : window) {
        Node& node = lookup(ref);
        std::stringstream ss;
        ss << "excl-exec:" << ref;
        solver_add(!(node.arch && node.trans), ss.str());
    }
}

void AEG::constrain_trans() {
    // NOTE: depends on results of construct_tfo()
    
    // transient execution of node requires incoming tfo edge
    for (const auto ref : node_range()) {
        Node& node = lookup(ref);
        const auto tfos = get_edges(Direction::IN, ref, Edge::TFO);
        // TODO: experiment with using z3::{atleast,exactly}(vec, 1) instead.
        const auto tfo_vec = z3::transform(context.context, tfos, [] (const auto& edge) -> z3::expr {
            return edge->exists;
        });
        const auto f = z3::exactly(tfo_vec, 1);
        node.constraints(z3::implies(node.trans, f), "trans-tfo");
    }
    
    // ensure that the number of transiently executed nodes doesn't exceed trans limit
    {
        z3::expr_vector trans {context.context};
        for (NodeRef ref : node_range()) {
            trans.push_back(lookup(ref).trans);
        }
        constraints(z3::atmost(trans, spec_depth), "trans-limit-max");
    }
}

void AEG::constrain_trans(const NodeRefSet& window, AssertFunc solver_add) {
    // transient execution of node requires incoming tfo edge
    for (NodeRef ref : window) {
        Node& node = lookup(ref);
        auto tfos = get_nodes(Direction::IN, ref, Edge::TFO);
        std::erase_if(tfos, [&window] (const auto& x) -> bool {
            return !window.contains(x.first);
        });
        const auto tfo_vec = z3::transform(context.context, tfos, [] (const auto& edge) -> z3::expr {
            return edge.second;
        });
        const auto f = z3::exactly(tfo_vec, 1);
        std::stringstream ss;
        ss << "trans-tfo:" << ref;
        solver_add(z3::implies(node.trans, f), ss.str());
    }
    
    // ensure that the number of transiently executed nodes doesn't exceed trans limit
    {
        z3::expr_vector trans {context};
        for (NodeRef ref : window) {
            trans.push_back(lookup(ref).trans);
        }
        solver_add(z3::atmost2(trans, spec_depth), "trans-limit-max");
    }
}

void AEG::constrain_tfo() {
    for (const NodeRef src : node_range()) {
        Node& src_node = lookup(src);
        
        z3::expr_vector tfos {context};
        for (const auto& edge : get_edges(Direction::OUT, src, Edge::TFO)) {
            tfos.push_back(edge->exists);
        }
        
        // add 'at most one tfo successor' constraint
        if (exits.find(src) == exits.end()) {
            src_node.constraints(z3::implies(src_node.exec(), z3::atmost2(tfos, 1)), "tfo-succ");
        }
    }
    
    // assert only one tfo window
    z3::expr_vector tfos {context.context};
    for_each_edge(Edge::TFO, [&] (const NodeRef src, const NodeRef dst, const Edge& edge) {
        const Node& src_node = lookup(src);
        const Node& dst_node = lookup(dst);
        tfos.push_back(src_node.arch && dst_node.trans && edge.exists);
    });
    constraints(z3::atmost2(tfos, 1), "at-most-one-spec-intro");
    
    // only one cold arch start
    z3::expr_vector cold_start {context.context};
    for (NodeRef ref : po.reverse_postorder()) {
        if (ref == entry || exits.contains(ref)) { continue; }
        const Node& node = lookup(ref);
        const auto tfo_ins = get_edges(Direction::IN, ref, Edge::TFO);
        const auto tfo_ins_v = z3::transform(context.context, tfo_ins, [] (const auto& e) -> z3::expr { return e->exists; });
        cold_start.push_back(node.arch && !z3::mk_or(tfo_ins_v));
    }
    constraints(z3::exactly(cold_start, 1), "one-cold-start");
}

void AEG::constrain_tfo(const NodeRefSet &window, AssertFunc solver_add) {
    for (const NodeRef src : window) {
        Node& src_node = lookup(src);
        
        z3::expr_vector tfos {context};
        for (const auto& edge : get_nodes(Direction::OUT, src, Edge::TFO)) {
            if (window.contains(edge.first)) {
                tfos.push_back(edge.second);
            }
        }
        
        if (!exits.contains(src)) {
            std::stringstream ss;
            ss << "tfo-succ:" << src;
            solver_add(z3::implies(src_node.exec(), z3::atmost2(tfos, 1)), ss.str());
        }
    }
    
    // assert only one tfo window
    z3::expr_vector tfos {context};
    for_each_edge(Edge::TFO, [&] (NodeRef src, NodeRef dst, const Edge& edge) {
        const Node& src_node = lookup(src);
        const Node& dst_node = lookup(dst);
        if (window.contains(src) && window.contains(dst)) {
            tfos.push_back(src_node.arch && dst_node.trans && edge.exists);
        }
    });
    solver_add(z3::atmost2(tfos, 1), "at-most-one-spec-intro");
    
    // only one cold arch start
    z3::expr_vector cold_start {context};
    for (NodeRef ref : po.reverse_postorder()) {
        if (ref == entry || exits.contains(ref) || !window.contains(ref)) { continue; }
        const Node& node = lookup(ref);
        const auto tfo_ins = get_nodes(Direction::IN, ref, Edge::TFO);
        z3::expr_vector tfo_ins_v {context};
        for (const auto& p : tfo_ins) {
            if (window.contains(p.first)) {
                tfo_ins_v.push_back(p.second);
            }
        }
        cold_start.push_back(node.arch && !z3::mk_or(tfo_ins_v));
    }
    solver_add(z3::exactly(cold_start, 1), "one-cold-start");
}

void AEG::constrain_comx() {
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        if (!node.is_special()) {
            if (node.can_xsaccess()) {
                node.constraints(*node.xstate == node.get_memory_address(), "xstate-addr-eq");
            }
        }
    }
}

void AEG::constrain_comx(const NodeRefSet& window, AssertFunc solver_add) {
    for (NodeRef ref : window) {
        Node& node = lookup(ref);
        if (!node.is_special()) {
            if (node.can_xsaccess()) {
                solver_add(*node.xstate == node.get_memory_address(), "xstate-addr-eq");
            }
        }
    }
}

}
