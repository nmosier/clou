#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>
#include <fstream>

#include "util/z3.h"
#include "aeg.h"
#include "config.h"
#include "fol.h"
#include "progress.h"
#include "timer.h"
#include "fork_work_queue.h"
#include "shm.h"
#include "cfg/expanded.h"
#include "util/iterator.h"

/* TODO
 * [ ] Don't use seen when generating tfo constraints
 * [ ] Use Graph<>'s node iterator, since it skips over deleted nodes? Or improve node range
 *     to skip deleted nodes.
 */

namespace aeg {

z3::solver AEG::make_solver() {
    z3::context& c = context.context;
    
    if (std::getenv("VERBOSE")) {
        z3::set_param("verbose", 10);
    }
    
    if (const char *s = std::getenv("PARALLEL")) {
        z3::set_param("parallel.enable", true);
        z3::set_param("parallel.threads.max", std::stoi(s));
    }

    std::vector<z3::tactic> tactics;
    
    z3::tactic acc {c, "skip"};
    const auto add = util::overloaded {
        [&] (const char *s) {
            acc = acc & z3::tactic(c, s);
        },
        [&] (const char *s, const z3::params& p) {
            acc = acc & z3::with(z3::tactic(c, s), p);
        }
    };
    
    if (const char *s_ = std::getenv("TACTICS")) {
        std::cerr << "using tactics " << s_ << "\n";
        char *s__ = ::strdup(s_);
        char *s = s__;
        while (char *tok = ::strsep(&s, " ")) {
            add(tok);
        }
        std::free(s__);
        return acc.mk_solver();
    } else {
        return z3::solver(c);
    }
}

void AEG::simplify() {
    Progress progress {nodes.size()};
    std::for_each(nodes.begin(), nodes.end(), [&] (Node& node) {
        node.simplify();
        ++progress;
    });
    progress.done();
    
    constraints.simplify();
    
    progress = Progress(nedges);
    graph.for_each_edge([&] (NodeRef, NodeRef, Edge& edge) {
        edge.simplify();
        ++progress;
    });
    progress.done();
}

void AEG::test() {
    unsigned naddrs = 0;
    for_each_edge(Edge::ADDR, [&] (NodeRef, NodeRef, const Edge&) {
        ++naddrs;
    });
    std::cerr << "Address edges: " << naddrs << "\n";
    if (naddrs > 0) {
#if 0
        std::ofstream ofs {"addrs.txt", std::ios_base::out | std::ofstream::app};
        ofs << lookup(1).inst->get_inst()->getFunction()->getName().str() << "\n";
#endif
    } else {
        return;
    }
    
    logv(3) << "testing...\n";
    
#if 0
    z3::solver solver {context.context};
#else
    z3::solver solver = make_solver();
#endif
    
    simplify();
    
    /* display stats */
    if (verbose >= 3) {
        auto& os = llvm::errs();
        os << constraints.exprs.size() << " top level constraints\n";
        const unsigned node_clauses =
        std::transform_reduce(nodes.begin(), nodes.end(), 0, std::plus<unsigned>(),
                              [] (const Node& node) {
            return node.constraints.exprs.size();
        });
        os << node_clauses << " node constraints\n";
        std::unordered_map<Edge::Kind, unsigned> edge_constraints;
        graph.for_each_edge([&] (NodeRef, NodeRef, const Edge& e) {
            edge_constraints[e.kind] += e.constraints.exprs.size();
        });
        os << std::transform_reduce(edge_constraints.begin(), edge_constraints.end(), 0, std::plus<unsigned>(), [] (const auto& pair) -> unsigned { return pair.second; }) << " edge constraints (total\n";
        for (const auto& pair : edge_constraints) {
            os << pair.first << " " << pair.second << "\n";
        }
    }
    
    // add edge constraints
    {
        std::cerr << __FUNCTION__ << ": adding edge constraints...\n";
        Progress progress {nedges};
        std::unordered_map<std::string, unsigned> names;
        graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
            edge.constraints.add_to(solver);
            ++progress;
        });
        progress.done();
    }
    
    // add node constraints
    {
        std::cerr << __FUNCTION__ << ": adding node constraints...\n";
        Progress progress {size()};
        for (NodeRef ref : node_range()) {
            lookup(ref).constraints.add_to(solver);
            ++progress;
        }
        progress.done();
    }
    
    // add main constraints
    constraints.add_to(solver);
    
    std::cerr << solver.statistics() << "\n";
    
    std::optional<Timer> timer = Timer();
    z3::scope scope {solver};
    timer = std::nullopt;
    
    // TODO: clean this crap up
    {
        Timer timer;
        leakage(solver);
    }
}

void AEG::add_unidir_edge(NodeRef src, NodeRef dst, const Edge& e) {
    if (e.possible()) {
        graph.insert(src, dst, e);
        ++nedges;
    }
}

void AEG::add_bidir_edge(NodeRef a, NodeRef b, const Edge& e) {
    Edge e1 = e;
    Edge e2 = e;
    const z3::expr dir = context.make_bool();
    e1.exists &=  dir;
    e2.exists &= !dir;
    add_unidir_edge(a, b, e1);
    add_unidir_edge(b, a, e2);
}

z3::expr AEG::add_optional_edge(NodeRef src, NodeRef dst, const Edge& e_, const std::string& name) {
    Edge e = e_;
    const z3::expr constr = e.exists;
    e.exists = context.make_bool(name);
    e.constraints(z3::implies(e.exists, constr), name);
    add_unidir_edge(src, dst, e);
    return e.exists;
}

namespace {
std::ostream& operator<<(std::ostream& os, const std::pair<z3::expr, std::string>& p) {
    return os << p.second << ":" << p.first;
}
}

std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& p : c.exprs) {
        os << p << " && ";
    }
    return os;
}

template <typename OutputIt>
OutputIt AEG::get_edges(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) {
    assert(!is_pseudoedge(kind));
    const auto& map = graph(dir);
    for (const auto& p : map.at(ref)) {
        for (auto& edge : p.second) {
            if (edge->kind == kind) {
                *out++ = edge.get();
            }
        }
    }
    return out;
}

AEG::EdgePtrVec AEG::get_edges(Direction dir, NodeRef ref, Edge::Kind kind) {
    EdgePtrVec es;
    get_edges(dir, ref, std::back_inserter(es), kind);
    return es;
}

std::vector<std::pair<NodeRef, z3::expr>> AEG::get_nodes(Direction dir, NodeRef ref, Edge::Kind kind) const {
    std::vector<std::pair<NodeRef, z3::expr>> res;
    get_nodes(dir, ref, std::back_inserter(res), kind);
    return res;
}


const AEG::Edge *AEG::find_edge(NodeRef src, NodeRef dst, Edge::Kind kind) const {
    assert(!is_pseudoedge(kind));
    const auto src_it = util::contains(graph.fwd, src);
    if (!src_it) { return nullptr; }
    const auto dst_it = util::contains((**src_it).second, dst);
    if (!dst_it) { return nullptr; }
    const auto& edges = (**dst_it).second;
    const auto it = std::find_if(edges.begin(), edges.end(), [=] (const auto& edgeptr) {
        return edgeptr->kind == kind;
    });
    return it == edges.end() ? nullptr : it->get();
}

AEG::Edge *AEG::find_edge(NodeRef src, NodeRef dst, Edge::Kind kind) {
    assert(!is_pseudoedge(kind));
    auto& edges = graph.fwd[src][dst];
    const auto it = std::find_if(edges.begin(), edges.end(), [kind] (const auto& edgeptr) {
        return edgeptr->kind == kind;
    });
    return it == edges.end() ? nullptr : it->get();
}

NodeRef AEG::add_node(Node&& node) {
    const NodeRef ref = size();
    nodes.push_back(std::move(node));
    graph.add_node(ref);
    return ref;
}

unsigned AEG::num_specs() const {
    return po.num_specs;
}


NodeRef AEG::exit_con(const z3::eval& eval) const {
    for (const NodeRef exit : exits) {
        if (eval(lookup(exit).arch)) {
            return exit;
        }
    }
    
    error("no arch exit!");
}

AEG::ValueLoc AEG::get_value_loc(NodeRef ref) const {
    return {
        *po.lookup(ref).id,
        lookup(ref).get_memory_address_pair().first,
    };
}

std::string AEG::function_name() const {
    return po.function_name();
}

}
