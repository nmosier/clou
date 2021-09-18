#include <vector>
#include <unordered_map>
#include <deque>
#include <unordered_set>
#include <fstream>

#include "z3-util.h"
#include "aeg.h"
#include "config.h"
#include "fol.h"
#include "progress.h"
#include "timer.h"

/* TODO
 * [ ] Don't use seen when generating tfo constraints
 * [ ] Use Graph<>'s node iterator, since it skips over deleted nodes? Or improve node range
 *     to skip deleted nodes.
 */

void AEG::dump_graph(const std::string& path) const {
    std::error_code ec;
    llvm::raw_fd_ostream os {path, ec};
    if (ec) {
        llvm::errs() << ec.message() << "\n";
        std::exit(1);
    }
    dump_graph(os);
}

void AEG::dump_graph(llvm::raw_ostream& os) const {
    os << R"=(
    digraph G {
    overlap = scale;
    splines = true;
    
    )=";
    
    // define nodes
    unsigned next_id = 0;
    std::unordered_map<NodeRef, std::string> names;
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        const std::string name = std::string("n") + std::to_string(next_id);
        names.emplace(ref, name);
        ++next_id;
        
        os << name << " ";
        
        std::stringstream ss;
        ss << node.inst.kind_tostr() << "\n";
        ss << node.inst << "\n";
        ss << "po: " << node.arch << "\n"
        << "tfo: " << node.trans << "\n"
        << "tfo_depth: " << node.trans_depth << "\n";
        
        if (node.addr_def) {
            ss << "addr (def): " << *node.addr_def << "\n";
        }
        if (!node.addr_refs.empty()) {
            ss << "addr (refs):";
            for (const auto& ref : node.addr_refs) {
                ss << " " << ref.second;
            }
            ss << "\n";
        }
        
        if (dump_constraints) {
            ss << "constraints: " << node.constraints << "\n";
        }
        
        dot::emit_kvs(os, "label", ss.str());
        os << ";\n";
    }
    
    // define edges
    graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
        os << names.at(src) << " -> " << names.at(dst) << " ";
        dot::emit_kvs(os, "label", util::to_string(edge));
        os << ";\n";
    });
    
    // graph labels (constraints)
    {
        os << "graph ";
        std::stringstream ss;
        ss << constraints;
        dot::emit_kvs(os, "label", ss.str());
        os << "\n";
    }
    
    os << "}\n";
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
        std::ofstream ofs {"addrs.txt", std::ios_base::out | std::ofstream::app};
        ofs << lookup(1).inst.I->getFunction()->getName().str() << "\n";
    } else {
        return;
    }
    
    logv(3) << "testing...\n";
    
    z3::solver solver {context.context};
    
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
            os << Edge::kind_tostr(pair.first) << " " << pair.second << "\n";
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
    
#define CHECKING 0
#if CHECKING
    // list of FOL expressions to evaulate
    const auto po = fol::edge_rel(*this, Edge::PO);
    const auto tfo = fol::edge_rel(*this, Edge::TFO);
    const auto rf = fol::edge_rel(*this, Edge::RF);
    const auto co = fol::edge_rel(*this, Edge::CO);
    const auto fr = fol::edge_rel(*this, Edge::FR);
    const auto com = rf + co + fr;
    const auto reads = fol::node_rel(*this, Inst::READ);
    const auto writes = fol::node_rel(*this, Inst::WRITE);
    const auto arch = fol::node_rel(*this, [] (const auto& p) -> z3::expr {
        return p.node.arch;
    });
    const auto trans = fol::node_rel(*this, [] (const auto& p) -> z3::expr {
        return p.node.trans;
    });
    const auto exec = arch + trans;
    const auto rfx = fol::edge_rel(*this, Edge::RFX);
    const auto cox = fol::edge_rel(*this, Edge::COX);
    const auto frx = fol::edge_rel(*this, Edge::FRX);
    const auto comx = rfx + cox + frx;
    
    const auto top = fol::node_rel(*this, Inst::ENTRY) & exec;
    const auto bot = fol::node_rel(*this, Inst::EXIT) & exec;
    
    const auto fr_computed = fol::join(fol::inverse(rf), co);
    const auto exprs = std::make_tuple(std::make_pair(po, "po"),
                                       std::make_pair(rf, "rf"),
                                       std::make_pair(co, "co"),
                                       std::make_pair(fr, "fr"),
                                       std::make_pair(fol::join(fol::inverse(rf), co), "fr'"),
                                       std::make_pair(bot, "bot"),
                                       std::make_pair(rfx, "rfx"),
                                       std::make_pair(cox, "cox"),
                                       std::make_pair(frx, "frx")
                                       );
#endif

    {
    Timer timer;
    solver.push();
    }

    {
        const auto addr_rel = fol::edge_rel(*this, Edge::ADDR);
        const auto trans_rel = fol::node_rel(*this, [&] (NodeRef, const Node& node) -> z3::expr {
            return node.trans;
        });
        const auto addr_expr = fol::some(fol::join(addr_rel, trans_rel), context.context);
        solver.push();
        solver.add(addr_expr);
        if (solver.check() == z3::sat) {
            output_execution("addr.dot", solver.get_model());
        } else {
            const auto exprs = solver.unsat_core();
            std::cerr << exprs << "\n";
            throw util::resume("no addr edges");
        }
        solver.pop();
        
        Timer timer;
        const auto nleaks = leakage(solver);
        std::cerr << "Detected " << nleaks << " leaks.\n";
        if (nleaks == 0) {
            return;
        }
    }
    
    unsigned nexecs = 0;
    
#if CHECKING
    const auto dump_expressions = [&] (const z3::model& model) {
        std::ofstream ofs {std::string("out/exec") + std::to_string(nexecs) + ".txt"};
        util::for_each_in_tuple(exprs, [&] (const auto& pair) {
            ofs << pair.second << ":\n";
            for (const auto& rel_pair : pair.first) {
                if (model.eval(rel_pair.second).is_true()) {
                    util::for_each_in_tuple(rel_pair.first, [&] (const auto& x) {
                        ofs << " " << x;
                    });
                    ofs << "\n";
                }
            }
            ofs << "\n";
        });
    };
#endif
    
    constexpr unsigned max_nexecs = 16;
    while (nexecs < max_nexecs) {
        Stopwatch timer;
        timer.start();
        const auto res = solver.check();
        timer.stop();
        std::cerr << res << " " << timer << "\n";
        
            switch (res) {
            case z3::unsat: {
                const auto& core = solver.unsat_core();
                for (const auto& expr : core) {
                    llvm::errs() << util::to_string(expr) << "\n";
                }
                goto done;
            }
            case z3::sat: {
                const z3::model model = solver.get_model();
                output_execution(std::string("out/exec") + std::to_string(nexecs) + ".dot", model);
#if CHECKING
                dump_expressions(model);
#endif
                
                ++nexecs;
                
                // add constraints
                std::cerr << "adding different solution constraints...\n";
                Stopwatch timer;
                timer.start();
                std::vector<z3::expr> exprs;
                auto it = std::back_inserter(exprs);
                for (const Node& node : nodes) {
                    *it++ = node.arch;
                    *it++ = node.trans;
                }
                
                for_each_edge([&] (NodeRef, NodeRef, const Edge& edge) {
                    *it++ = edge.exists;
                });
                
                const z3::expr same_sol = std::transform_reduce(exprs.begin(), exprs.end(), context.TRUE, util::logical_and<z3::expr>(), [&] (const z3::expr& e) -> z3::expr {
                    return e == model.eval(e);
                });
                
                solver.add(!same_sol);
                
                timer.stop();
                std::cerr << timer << "\n";
                
                break;
            }
            case z3::unknown:
                goto done;
        }
    }
    
done:
    std::cerr << "found " << nexecs << " executions\n";
    
    solver.pop();
    
#if CHECKING
    // check FOL assertions
    const auto po_closure = fol::irreflexive_transitive_closure(po);
    
    const std::vector<std::tuple<z3::expr, z3::check_result, std::string>> vec = {
        {!fol::for_all(exec - bot, fol::for_func<NodeRef> {[&] (const fol::relation<NodeRef>& node) -> z3::expr {
            return fol::one(fol::join(node, tfo), context.context);
        }}, context.context), z3::unsat, "tfo succ"},
        {!fol::for_all(exec - top, fol::for_func<NodeRef> {[&] (const fol::relation<NodeRef>& node) -> z3::expr {
            return fol::one(fol::join(tfo, node), context.context);
        }}, context.context), z3::unsat, "tfo pred"},
        {!fol::one(exec & top, context.context), z3::unsat, "one top"},
        {!fol::one(exec & bot, context.context), z3::unsat, "one bot"},
        {fol::no(rf, context.context), z3::unsat, "no rf"},
        {fol::no(reads, context.context), z3::unsat, "no reads"},
        {fol::some(arch, context.context), z3::sat, "some arch"},
        {!fol::subset(fol::join(co, co), co, context.context), z3::unsat, "co.co in co"},
        {!fol::equal(fr, fol::join(fol::inverse(rf), co), context.context), z3::unsat, "fr = ~rf.co"},
        {!fol::subset(po, po_closure, context.context), z3::unsat, "po in ^po"},
        {!fol::acyclic(po, context.context), z3::unsat, "acyclic[po]"},
        {!fol::acyclic(po + com, context.context), z3::unsat, "acyclic[po+com]"},
        {!fol::equal(rf, fol::inverse(fol::inverse(rf)), context.context), z3::unsat, "rf = ~~rf"},
        {!fol::acyclic(cox, context.context), z3::unsat, "acyclic[cox]"},
        {!fol::acyclic(rfx + cox, context.context), z3::unsat, "acyclic[rfx+cox]"},
        {!fol::acyclic(comx, context.context), z3::unsat, "acyclic[comx]"},
        {fol::some(trans, context.context), z3::sat, "some trans"},
    };
    
    unsigned passes = 0;
    unsigned fails = 0;
    for (const auto& task : vec) {
        solver.push();
        solver.add(std::get<0>(task));
        const auto res = solver.check();
        if (res != std::get<1>(task)) {
            std::cerr << "CHECK FAILED: " << std::get<2>(task) << "\n";
            ++fails;
            if (res == z3::sat) {
                // produce model
                const z3::model model = solver.get_model();
                output_execution(std::string("out/exec") + std::to_string(nexecs) + ".dot", model);
                dump_expressions(model);
                ++nexecs;
            } else if (res == z3::unsat) {
                const auto& core = solver.unsat_core();
                for (const auto& expr : core) {
                    llvm::errs() << util::to_string(expr) << "\n";
                }
            }
        } else {
            ++passes;
        }
        solver.pop();
    }
    
    std::cerr << "Passes: " << passes << "\n"
              << "Fails:  " << fails  << "\n";
#endif

}

void AEG::add_unidir_edge(NodeRef src, NodeRef dst, const UHBEdge& e) {
    if (e.possible()) {
        graph.insert(src, dst, e);
        ++nedges;
    }
}

void AEG::add_bidir_edge(NodeRef a, NodeRef b, const UHBEdge& e) {
    UHBEdge e1 = e;
    UHBEdge e2 = e;
    const z3::expr dir = context.make_bool();
    e1.exists &=  dir;
    e2.exists &= !dir;
    add_unidir_edge(a, b, e1);
    add_unidir_edge(b, a, e2);
}

void AEG::add_optional_edge(NodeRef src, NodeRef dst, const UHBEdge& e_, const std::string& name) {
    UHBEdge e = e_;
    const z3::expr constr = e.exists;
    e.exists = context.make_bool(name);
    e.constraints(z3::implies(e.exists, constr), name);
    add_unidir_edge(src, dst, e);
}

namespace {
std::ostream& operator<<(std::ostream& os, const std::pair<z3::expr, std::string>& p) {
    return os << p.second << ":" << p.first;
}
}

std::ostream& operator<<(std::ostream& os, const UHBConstraints& c) {
    for (const auto& p : c.exprs) {
        os << p << " && ";
    }
    return os;
}

void AEG::output_execution(std::ostream& os, const z3::model& model) const {
    os << R"=(
    digraph G {
    overlap = scale;
    splines = true;
    
    )=";
    
    // define nodes
    unsigned next_id = 0;
    std::unordered_map<NodeRef, std::string> names;
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (model.eval(node.exec()).is_true()) {
            const std::string name = std::string("n") + std::to_string(next_id);
            names.emplace(ref, name);
            ++next_id;
            
            os << name << " ";
            std::stringstream ss;
            ss << ref << " " << node.inst << "\n";
            
            switch (node.inst.kind) {
                case Inst::WRITE:
                case Inst::READ:
                    ss << "{" << model.eval(node.get_memory_address()) << "} ";
                    break;
                default: break;
            }
            
            if (model.eval(node.xsread).is_true()) {
                ss << "xsread " << model.eval(node.xsread_order) << " ";
            }
            if (model.eval(node.xswrite).is_true()) {
                ss << "xswrite " << model.eval(node.xswrite_order) << " ";
            }
            
#if 0
            if (model.eval(node.trans).is_true()) {
                ss << "[" << model.eval(node.trans_group_min) << " " << model.eval(node.trans_group_max) << "] ";
            }
#endif
            
            std::string color;
            if (model.eval(node.arch).is_true()) {
                color = "green";
            } else if (model.eval(node.trans).is_true()) {
                color = "red";
            }
            
            dot::emit_kvs(os, dot::kv_vec {{"label", ss.str()}, {"color", color}});
            os << ";\n";
        }
    }
    
    const auto output_edge = [&] (NodeRef src, NodeRef dst, Edge::Kind kind) {
        if (!include_edges.empty()) {
            if (include_edges.find(kind) == include_edges.end()) { return; }
        }
        
        os << names.at(src) << " -> " << names.at(dst) << " ";
        static const std::unordered_map<Edge::Kind, std::string> colors = {
            {Edge::TFO, "black"},
            {Edge::RF, "gray"},
            {Edge::CO, "blue"},
            {Edge::FR, "purple"},
            {Edge::RFX, "gray"},
            {Edge::COX, "blue"},
            {Edge::FRX, "purple"},
            {Edge::ADDR, "red"},
        };
        const std::string& color = colors.at(kind);
        dot::emit_kvs(os, dot::kv_vec {{"label", util::to_string(kind)}, {"color", color}});
        os << ";\n";
    };
    
    graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
        if (model.eval(edge.exists).is_true()) {
            output_edge(src, dst, edge.kind);
        }
    });
    
    // output pseudo com and comx
    using EdgeSig = std::tuple<NodeRef, NodeRef, Edge::Kind>;
    using EdgeSigVec = std::vector<EdgeSig>;
    EdgeSigVec com, comx;
    get_concrete_com(model, std::back_inserter(com));
    get_concrete_comx(model, std::back_inserter(comx));
    for (const auto& edge : com) {
        std::apply(output_edge, edge);
    }
    for (const auto& edge : comx) {
        std::apply(output_edge, edge);
    }
    
    os << "}\n";
}

void AEG::output_execution(const std::string& path, const z3::model& model) const {
    std::ofstream ofs {path};
    output_execution(ofs, model);
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

AEG::EdgePtrVec AEG::get_edges(Direction dir, NodeRef ref, UHBEdge::Kind kind) {
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

z3::expr AEG::edge_exists(NodeRef src, NodeRef dst, Edge::Kind kind) const {
    switch (kind) {
        case Edge::CO:
            return co_exists(src, dst);
            
        default:
            if (const Edge *edge = find_edge(src, dst, kind)) {
                return edge->exists;
            } else {
                return context.FALSE;
            }
    }
}

NodeRef AEG::add_node(const Node& node) {
    const NodeRef ref = size();
    nodes.push_back(node);
    graph.add_node(ref);
    return ref;
}
