#pragma once

#include <string>
#include <cassert>
#include <unordered_set>
#include <sstream>
#include <set>

#include <llvm/IR/Instruction.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <z3++.h>

#include "z3-util.h"
#include "aeg-po2.h"
#include "graph.h"
#include "util.h"
#include "uhb.h"
#include "noderef.h"
#include "progress.h"
#include "taint.h"

class AEG {
public:
    const AEGPO& po; /*!<  The input CFG. The AEG constructs nodes in a 1:1 correspondence and heavily uses the preds/succs relations of this CFG. */
    UHBContext context; /*!<  The context for AEG construction. */

    using Node = UHBNode;
    using Edge = UHBEdge;
    using graph_type = Graph<NodeRef, Edge, std::hash<NodeRef>, Edge::Hash>;
    
    NodeRef entry; /*!< The unique entry node of the AEG (node has type Inst::Kind::ENTRY) */
    NodeRefSet exits; /*!< The set of exit nodes of the AEG (nodes have type Inst::Kind::EXIT) */
    
    graph_type graph;
    
    /** Construct the AEG.
     * \param AA alias analysis results to use
     * \param rob_size reorder buffer size (used to limit comx)
     */
    void construct(llvm::AliasAnalysis& AA, unsigned rob_size);
    
    const Node& lookup(NodeRef ref) const { return nodes.at(static_cast<unsigned>(ref)); }
    Node& lookup(NodeRef ref) { return nodes.at(static_cast<unsigned>(ref)); }
    
    explicit AEG(const AEGPO& po): po(po), context(), constraints() {}
    
    void dump_graph(llvm::raw_ostream& os) const;
    void dump_graph(const std::string& path) const;
    
    void simplify();
    
    void test();
    
    std::size_t size() const { return nodes.size(); } /*!< Get the number of nodes in the graph */
    
    template <typename Function> void for_each_edge(Function f) { graph.for_each_edge(f); }
    template <typename Function> void for_each_edge(Function f) const { graph.for_each_edge(f); }
    template <typename Function> void for_each_edge(Edge::Kind kind, Function f);
    template <typename Function> void for_each_edge(Edge::Kind kind, Function f) const;
    
    template <typename Function> void for_each_node(Inst::Kind kind, Function f);
    template <typename Function> void for_each_node(Inst::Kind kind, Function f) const;
    
    const UHBContext& ctx() const { return context; }
   
private:
    UHBConstraints constraints; /*!< global constraints (i.e. can't be attributed to one particular node or edge) */
    std::vector<Node> nodes;
    unsigned nedges = 0;
    
    unsigned num_specs() const { return po.num_specs; }
    
    void construct_nodes();
    void construct_exec();
    void construct_arch();
    void construct_trans();
    void construct_po();
    void construct_tfo();
    void construct_tfo2();
    void construct_addr_defs();
    void construct_addr_refs();
    void construct_aliases(llvm::AliasAnalysis& AA);
    void construct_com();
    void construct_arch_order();
    void construct_exec_order();
    void construct_trans_group();
    void construct_xsaccess_order(const NodeRefSet& xsaccesses);
    void construct_mem();
    void construct_comx();
    void construct_addr();
    
    template <typename OutputIt>
    void leakage_rfx(NodeRef read, z3::solver& solver, OutputIt out) const;
    template <typename OutputIt>
    void leakage_cox(NodeRef write, z3::solver& solver, OutputIt out) const;
    template <typename OutputIt>
    void leakage_frx(NodeRef write, z3::solver& solver, OutputIt out) const;

    z3::expr leakage_rfx2() const;
    z3::expr leakage_cox2() const;
    z3::expr leakage_frx2() const;
    unsigned leakage2(z3::solver& solver, unsigned max) const;
    
    /** Detect all 3 kinds of leakage in the AEG.
     * \param solver Solver to use. This solver should already have all the constraints added.
     * \return Garbage
     */
    unsigned leakage(z3::solver& solver);
    
public:
    using NodeRange = util::RangeContainer<NodeRef>;
    NodeRange node_range() const {
        return NodeRange {NodeRef {entry}, NodeRef {static_cast<unsigned>(nodes.size())}};
    }
    
private:
    NodeRef add_node(const Node& node); /*!< Add node to the AEG and return its corresponding node ref. */
    
    void add_unidir_edge(NodeRef src, NodeRef dst, const UHBEdge& e); /*!< Add a directed edge from \p src to \p dst with edge \p e */
    void add_bidir_edge(NodeRef a, NodeRef b, const UHBEdge& e); /*!< Add a directed edge in each direction between nodes \p a and \p b with edge \p e */
    /** Add a directed edge from \p src to \p dst and make the edge \p e optional with an additional unconstrained boolean variable
     * @param name The base name of the unconstrained boolean variable
     */
    z3::expr add_optional_edge(NodeRef src, NodeRef dst, const UHBEdge& e, const std::string& name);
    
    template <typename OutputIt>
    OutputIt get_edges(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind);
    
    using EdgePtrVec = std::vector<Edge *>;
    EdgePtrVec get_edges(Direction dir, NodeRef ref, Edge::Kind kind);
    
    template <typename OutputIt>
    OutputIt get_nodes(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) const;
    
    std::vector<std::pair<NodeRef, z3::expr>> get_nodes(Direction dir, NodeRef ref, Edge::Kind kind) const;
    
    void output_execution(std::ostream& os, const z3::model& model);
    void output_execution(const std::string& path, const z3::model& model);
    
    Edge *find_edge(NodeRef src, NodeRef dst, Edge::Kind kind);
    const Edge *find_edge(NodeRef src, NodeRef dst, Edge::Kind kind) const;
    z3::expr edge_exists(NodeRef src, NodeRef dst, Edge::Kind kind) const;
    
    template <typename Pred>
    z3::expr cyclic(Pred pred) const;
    
    template <typename Pred>
    z3::expr acyclic(Pred pred) const { return !cyclic(pred); }
    
    // acyclicity check using integers
    template <typename Pred>
    z3::expr acyclic_int(Pred pred);
    
    bool is_pseudoedge(Edge::Kind kind) const {
        static const std::unordered_set<Edge::Kind> pseudo = {
            Edge::RF, Edge::CO, Edge::FR, Edge::RFX, Edge::COX, Edge::FRX,
        };
        return pseudo.find(kind) != pseudo.end();
    }
    
public:
    template <typename OutputIt, typename Pred>
    OutputIt get_nodes_if(OutputIt out, Pred pred) const {
        for (NodeRef ref : node_range()) {
            if (pred(ref, lookup(ref))) {
                *out++ = ref;
            }
        }
        return out;
    }
    
    template <typename Container, typename OutputIt>
    OutputIt get_nodes(OutputIt out, const Container& container) const {
        return get_nodes_if(out, [&] (NodeRef, const Node& node) -> bool {
            return container.find(node.inst.kind) != container.end();
        });
    }
    
    template <typename OutputIt>
    OutputIt get_nodes(OutputIt out, Inst::Kind kind) {
        return get_nodes(out, std::unordered_set<Inst::Kind> {kind});
    }
    
    template <typename OutputIt>
    OutputIt get_writes(OutputIt out) const {
        return get_nodes(out, std::unordered_set<Inst::Kind> {Inst::ENTRY, Inst::WRITE});
    }
    
    template <typename OutputIt>
    OutputIt get_reads(OutputIt out) const {
        return get_nodes(out, std::unordered_set<Inst::Kind> {Inst::EXIT, Inst::READ});
    }
    
    z3::expr co_exists(NodeRef src, NodeRef dst) const {
        const Node& srcn = lookup(src);
        const Node& dstn = lookup(dst);
        const z3::expr f = context.bool_val(srcn.arch_order < dstn.arch_order) && srcn.arch && dstn.arch && srcn.same_addr(dstn);
        return f.simplify();
    }
    
    // NOTE: We don't include an rf_exists() predicate since this would be expensive to compute directly.
    
    z3::expr rfx_exists(NodeRef src, NodeRef dst) const;
    z3::expr cox_exists(NodeRef src, NodeRef dst) const;
    z3::expr frx_exists(NodeRef src, NodeRef dst) const;


    template <typename T>
    class Dataflow;
    
private:
    using ValueLoc = std::pair<AEGPO::ID, const llvm::Value *>;
    using ValueLocRel = std::unordered_map<std::pair<ValueLoc, ValueLoc>, llvm::AliasResult>;
    ValueLocRel alias_rel;
    
    llvm::AliasResult check_alias(NodeRef ref1, NodeRef ref2) const;
    void add_alias_result(const ValueLoc& vl1, const ValueLoc& vl2, llvm::AliasResult res);
    
    // SPECULATION QUERIES
    bool can_trans(NodeRef ref, NodeRef& pred) const {
        const auto& preds = po.po.rev.at(ref);
        if (preds.size() == 1) {
            pred = *preds.begin();
            return true;
        } else {
            return false;
        }
    }
    
    bool can_trans(NodeRef ref) const {
        NodeRef pred;
        return can_trans(ref, pred);
    }
    
    bool can_introduce_trans(NodeRef ref) const {
        return po.po.fwd.at(ref).size() > 1;
    }
    
    // for output execution purposes
    template <typename OutputIt>
    OutputIt get_concrete_comx(const z3::model& model, OutputIt out) const;
    
    template <typename OutputIt>
    OutputIt get_concrete_com(const z3::model& model, OutputIt out) const;
    
    friend class Taint;
    friend class Taint_Array;
    std::unique_ptr<Taint> tainter;
};

template <typename Function>
void AEG::for_each_edge(Edge::Kind kind, Function f) {
    graph.for_each_edge([&] (NodeRef src, NodeRef dst, Edge& edge) {
        if (edge.kind == kind) {
            f(src, dst, edge);
        }
    });
}

template <typename Function>
void AEG::for_each_edge(Edge::Kind kind, Function f) const {
    graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
        if (edge.kind == kind) {
            f(src, dst, edge);
        }
    });
}

template <typename Pred>
z3::expr AEG::cyclic(Pred pred) const {
    std::vector<graph_type::Cycle> cycles;
    graph.cycles(std::back_inserter(cycles), pred);
    
    const auto op_edges = [&] (const std::vector<Edge>& edges) -> z3::expr {
        return std::transform_reduce(edges.begin(), edges.end(), context.FALSE, util::logical_or<z3::expr>(),
                                     [] (const Edge& e) {
            return e.exists;
        });
    };
    
    const auto op_cycle = [&] (const graph_type::Cycle& cycle) -> z3::expr {
        return std::transform_reduce(cycle.edges.begin(), cycle.edges.end(), context.TRUE, util::logical_and<z3::expr>(), op_edges);
    };
    
    const auto op_cycles = [&] (const std::vector<graph_type::Cycle>& cycles) -> z3::expr {
        return std::transform_reduce(cycles.begin(), cycles.end(), context.FALSE, util::logical_or<z3::expr>(), op_cycle);
    };
    
    return op_cycles(cycles);
}

template <typename Pred>
z3::expr AEG::acyclic_int(Pred pred) {
    z3::expr acc = context.TRUE;
    std::unordered_map<NodeRef, z3::expr> ints;
    const auto lookup_int = [&] (NodeRef ref) -> z3::expr {
        auto it = ints.find(ref);
        if (it == ints.end()) {
            it = ints.emplace(ref, context.make_int("acyclic")).first;
        }
        return it->second;
        
    };
    for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
        if (pred(edge)) {
            const z3::expr src_int = lookup_int(src), dst_int = lookup_int(dst);
            const z3::expr f = z3::implies(edge.exists, src_int < dst_int);
            acc = acc && f;
        }
    });
    return acc;
}


template <typename OutputIt>
OutputIt AEG::get_nodes(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) const {
    const auto& map = graph(dir);
    for (const auto& p : map.at(ref)) {
        for (const auto& edge : p.second) {
            if (edge->kind == kind) {
                *out++ = std::make_pair(p.first, edge->exists);
                break;
            }
        }
    }
    return out;
}

template <typename T>
class AEG::Dataflow {
public:
    using Map = std::unordered_map<NodeRef, T>;
    struct Result {
        Map in;
        Map out;
    };
    
    enum Direction {
        FWD, REV
    };
    
    Dataflow() {}
    
    template <typename Transfer, typename Meet>
    Dataflow(const AEG& aeg, const T& top, Direction dir, Result& res, Transfer transfer, Meet meet) {
        res = (*this)(aeg, top, dir, transfer, meet);
    }
    
    template <typename Transfer, typename Meet>
    Result operator()(const AEG& aeg, const T& top, Direction dir, Transfer transfer, Meet meet) const {
        Result res;
        
        std::vector<NodeRef> order;
        const AEGPO::Rel::Map *rel;
        switch (dir) {
            case Direction::FWD:
                aeg.po.reverse_postorder(std::back_inserter(order));
                rel = &aeg.po.po.rev;
                break;
            case Direction::REV:
                aeg.po.postorder(std::back_inserter(order));
                rel = &aeg.po.po.fwd;
                break;
        }
        
        for (NodeRef ref : order) {
            T& in = res.in.emplace(ref, top).first->second;
            for (NodeRef pred : rel->at(ref)) {
                in = meet(in, res.out.at(pred));
            }
            res.out.emplace(ref, transfer(ref, in));
        }
        
        return res;
    }
    
private:
};

template <typename OutputIt>
OutputIt AEG::get_concrete_comx(const z3::model& model, OutputIt out) const {
    using SymLess = UHBNode::xsaccess_order_less;
    struct Less {
        SymLess sym;
        const z3::model& model;
        Less(const AEG& aeg, const z3::model& model): sym(aeg), model(model) {}
        bool operator()(NodeRef a, NodeRef b) const {
            return z3::to_bool(model.eval(sym(a, b), true));
        }
    };
    
    const Less less {*this, model};
    using Set = std::set<NodeRef, Less>;
    Set reads {less};
    Set writes {less};
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (z3::to_bool(model.eval(node.exec() && node.xsread, true))) {
            [[maybe_unused]] const auto res = reads.insert(ref);
            assert(res.second);
        }
        if (z3::to_bool(model.eval(node.exec() && node.xswrite, true))) {
            [[maybe_unused]] const auto res = writes.insert(ref);
            assert(res.second);
        }
    }
    
    assert(!reads.empty());
    assert(!writes.empty());
    
    const auto same_xstate = [&] (NodeRef a, NodeRef b) -> bool {
        return model.eval(lookup(a).same_xstate(lookup(b)));
    };
    
    // rfx
    for (const NodeRef read : reads) {
        if (entry == read) {
        } else if (exits.find(read) == exits.end()) {
            // not an exit
            const auto write_rbegin = std::make_reverse_iterator(writes.lower_bound(read));
            const auto write_rit = std::find_if(write_rbegin, writes.rend(), [&] (NodeRef write) -> bool {
                return same_xstate(read, write);
            });
            assert(write_rit != writes.rend());
            *out++ = std::make_tuple(*write_rit, read, Edge::RFX);
        } else {
            // is an exit
            std::unordered_map<z3::expr, NodeRef> mem;
            for (auto it = writes.begin(); it != writes.end(); ++it) {
                if (*it != entry) {
                    const z3::expr xstate = model.eval(lookup(*it).xstate);
                    mem[xstate] = *it;
                }
            }
            for (const auto& p : mem) {
                *out++ = std::make_tuple(p.second, read, Edge::RFX);
            }
        }
    }
    
    // cox
    for (auto it1 = writes.begin(); it1 != writes.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != writes.end(); ++it2) {
            if (same_xstate(*it1, *it2)) {
                *out++ = std::make_tuple(*it1, *it2, Edge::COX);
            }
        }
    }
    
    // frx
    for (auto read_it = reads.begin(); read_it != reads.end(); ++read_it) {
        const auto write_begin = writes.upper_bound(*read_it);
        for (auto write_it = write_begin; write_it != writes.end(); ++write_it) {
            if (same_xstate(*read_it, *write_it)) {
                *out++ = std::make_tuple(*read_it, *write_it, Edge::FRX);
            }
        }
    }
    
    return out;
}


template <typename OutputIt>
OutputIt AEG::get_concrete_com(const z3::model& model, OutputIt out) const {
    // get po execution stream
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    NodeRefVec path;
    std::copy_if(order.begin(), order.end(), std::back_inserter(path), [&] (NodeRef ref) -> bool {
        return model.eval(lookup(ref).arch).is_true();
    });
    
    // rf
    std::unordered_map<z3::expr, NodeRef> mem;
    for (const NodeRef ref : path) {
        const Node& node = lookup(ref);
        switch (node.inst.kind) {
            case Inst::READ: {
                const z3::expr addr = model.eval(lookup(ref).get_memory_address());
                const auto mem_it = mem.find(addr);
                const NodeRef src = mem_it == mem.end() ? entry : mem_it->second;
                *out++ = std::make_tuple(src, ref, Edge::RF);
                break;
            }
                
            case Inst::WRITE: {
                const z3::expr addr = model.eval(lookup(ref).get_memory_address());
                mem[addr] = ref;
                break;
            }
                
            case Inst::ENTRY: break;
            case Inst::EXIT: {
                for (const auto& p : mem) {
                    *out++ = std::make_tuple(p.second, ref, Edge::RF);
                }
                break;
            }
                
            case Inst::OTHER: break;
            default: std::abort();
        }
    }
    
    // co
    for (auto it1 = path.begin(); it1 != path.end(); ++it1) {
        const Node& n1 = lookup(*it1);
        if (!n1.is_write()) { continue; }
        for (auto it2 = std::next(it1); it2 != path.end(); ++it2) {
            const Node& n2 = lookup(*it2);
            if (!n2.is_write()) { continue; }
            if (model.eval(n1.same_addr(n2)).is_false()) { continue; }
            *out++ = std::make_tuple(*it1, *it2, Edge::CO);
        }
    }
    
    // fr
    for (auto read_it = path.begin(); read_it != path.end(); ++read_it) {
        const Node& read = lookup(*read_it);
        if (!read.is_read()) { continue; }
        for (auto write_it = std::next(read_it); write_it != path.end(); ++write_it) {
            const Node& write = lookup(*write_it);
            if (!write.is_write()) { continue; }
            if (model.eval(read.same_addr(write)).is_false()) { continue; }
            *out++ = std::make_tuple(*read_it, *write_it, Edge::FR);
        }
    }
    
    return out;
}

template <typename Function> void AEG::for_each_node(Inst::Kind kind, Function f) {
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        if (node.inst.kind == kind) {
            f(ref, node);
        }
    }
}

template <typename Function> void AEG::for_each_node(Inst::Kind kind, Function f) const {
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (node.inst.kind == kind) {
            f(ref, node);
        }
    }
}
