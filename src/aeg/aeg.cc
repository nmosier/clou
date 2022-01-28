#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>
#include <fstream>

#include "util/z3.h"
#include "aeg.h"
#include "config.h"
#include "util/progress.h"
#include "util/timer.h"
#include "cfg/expanded.h"
#include "util/iterator.h"
#include "util/algorithm.h"
#include "aeg/edge.h"

namespace aeg {

bool AEG::may_source_stb(NodeRef load, NodeRef store) const {
    return !stb_size || lookup(load).stores_in < lookup(store).stores_in + static_cast<int>(*stb_size);
}

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

z3::expr AEG::same_addr(NodeRef a, NodeRef b) const {
    const Node& a_node = lookup(a);
    const Node& b_node = lookup(b);
    if (a_node.is_special() || b_node.is_special()) {
        return context.TRUE;
    } else {
        return get_memory_address(a) == get_memory_address(b);
    }
}

z3::expr AEG::get_memory_address(NodeRef ref) const {
    const Node& node = lookup(ref);
    const Address& addr = node.get_memory_address();
    
    z3::expr trans = context.FALSE;
    
    switch (leakage_class) {
        case LeakageClass::SPECTRE_V1: {
            // check for transient ADDR_GEP
            for (const auto& p : get_nodes(Direction::IN, ref, Edge::Kind::ADDR_GEP)) {
                trans = trans || ((node.trans || lookup(p.first).trans) && p.second);
            }
            break;
        }
            
        case LeakageClass::SPECTRE_V4: {
            // check if load
            trans = node.trans && node.read;
            break;
        }
            
        default: std::abort();
    }
    
    return z3::ite(trans, addr.trans, addr.arch).simplify();
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
OutputIt AEG::get_edges(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) const {
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

AEG::EdgePtrVec AEG::get_edges(Direction dir, NodeRef ref, Edge::Kind kind) const {
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


void AEG::test(TransmitterOutputIt out) {
    unsigned naddrs = 0;
    for_each_edge(Edge::ADDR, [&] (NodeRef, NodeRef, const Edge&) {
        ++naddrs;
    });
    std::cerr << "Address edges: " << naddrs << "\n";
    if (naddrs == 0) {
        return;
    }
    
    logv(1, "testing...\n");
    
    z3::solver solver = make_solver();
    
    simplify();
    
    /* display stats */
    if (verbose >= 0) {
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
    
    std::unordered_map<std::string, unsigned> hist;
    
    {
        Timer timer;
        
#if 0
        // add edge constraints
        {
            logv(0, __FUNCTION__ << ": adding edge constraints...\n");
            Progress progress {nedges};
            std::unordered_map<std::string, unsigned> names;
            graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
                edge.constraints.add_to(solver);
                edge.constraints.dump(hist);
                ++progress;
            });
            progress.done();
        }
#endif
        
        // add node constraints
        {
            logv(0, __FUNCTION__ << ": adding node constraints...\n");
            Progress progress {size()};
            for (NodeRef ref : node_range()) {
                lookup(ref).constraints.add_to(solver);
                lookup(ref).constraints.dump(hist);
                ++progress;
            }
            progress.done();
        }
        
        // add main constraints
        logv(0, __FUNCTION__ << ": adding main constraints...");
        constraints.add_to_progress(solver);
        constraints.dump(hist);

#if 0
        // add extra constraints
        NodeRefSet noderefs;
        for (NodeRef ref : node_range()) {
            noderefs.insert(ref);
        }
        
        constrain_arch(noderefs, solver);
        constrain_exec(noderefs, solver);
        constrain_tfo(noderefs, solver);
        constrain_comx(noderefs, solver);
#endif

        logv(0, __FUNCTION__ << ": added constraints ");
    }
    
    std::cerr << "NAMED ASSERTIONS HISTOGRAM:\n";
    for (const auto& p : hist) {
        std::cerr << p.first << " " << p.second << "\n";
    }
    
    {
        Timer timer;
        leakage(solver, out);
        std::cerr << "ANALYZED_test: " << cpu_time() << "\n";
    }
}

}

