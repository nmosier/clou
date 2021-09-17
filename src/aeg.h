#pragma once

#include <string>
#include <cassert>
#include <unordered_set>
#include <sstream>

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

class AEG {
private:
    const AEGPO& po;
    UHBContext context; // this is because all other members depend on this context
public:
    using Node = UHBNode;
    using Edge = UHBEdge;
    using graph_type = Graph<NodeRef, Edge, std::hash<NodeRef>, Edge::Hash>;
    
    NodeRef entry;
    NodeRefSet exits;
    // NodeRef exit;
    
    graph_type graph;
    
    void construct(unsigned spec_depth, llvm::AliasAnalysis& AA);
    
    const Node& lookup(NodeRef ref) const { return nodes.at(static_cast<unsigned>(ref)); }
    Node& lookup(NodeRef ref) { return nodes.at(static_cast<unsigned>(ref)); }
    
    explicit AEG(const AEGPO& po): po(po), context(), constraints() {}
    
    void dump_graph(llvm::raw_ostream& os) const;
    void dump_graph(const std::string& path) const;
    
    void simplify();
    
    void test();
    
    std::size_t size() const { return nodes.size(); }
    
    using Path = std::vector<NodeRef>;
    
    template <typename Function> void for_each_edge(Function f) { graph.for_each_edge(f); }
    template <typename Function> void for_each_edge(Function f) const { graph.for_each_edge(f); }
    template <typename Function> void for_each_edge(Edge::Kind kind, Function f);
    template <typename Function> void for_each_edge(Edge::Kind kind, Function f) const;
    
    template <typename Function> void for_each_node(Inst::Kind kind, Function f);
    template <typename Function> void for_each_node(Inst::Kind kind, Function f) const;
    
    const UHBContext& ctx() const { return context; }
   
private:
    UHBConstraints constraints;
    std::vector<Node> nodes;
    unsigned nedges = 0;
    
    unsigned num_specs() const { return po.num_specs; }
    
    using ClosureMap = std::unordered_map<NodeRef, std::unordered_set<NodeRef>>;
    template <typename Pred>
    ClosureMap find_predecessors(Pred f) const;
    
    void construct_nodes();
    void construct_exec();
    void construct_arch();
    void construct_trans();
    void construct_po();
    void construct_tfo();
    void construct_addr_defs();
    void construct_addr_refs();
    void construct_aliases(llvm::AliasAnalysis& AA);
    void construct_com();
    void construct_exec_order();
    void construct_trans_group();
    void construct_xsaccess_order(const NodeRefSet& xsreads, const NodeRefSet& xswrites);
    void construct_mem();
    
    typedef void construct_com_f(const NodeRefVec& reads, const NodeRefVec& writes,
                                 const ClosureMap& pred_reads, const ClosureMap& pred_writes);
    void construct_co(const NodeRefVec& reads, const NodeRefVec& writes,
                      const ClosureMap& pred_reads, const ClosureMap& pred_writes);
    void construct_rf(const NodeRefVec& reads, const NodeRefVec& writes,
                      const ClosureMap& pred_reads, const ClosureMap& pred_writes);
    void construct_fr(const NodeRefVec& reads, const NodeRefVec& writes,
                      const ClosureMap& pred_reads, const ClosureMap& pred_writes);
    void construct_comx();
    void construct_rfx(const NodeRefSet& xreads, const NodeRefSet& xwrites);
    void construct_cox(const NodeRefSet& xreads, const NodeRefSet& xwrites);
    void construct_frx(const NodeRefSet& xreads, const NodeRefSet& xwrites);
    void construct_addr();
    
    void leakage(NodeRef src, NodeRef dst, const Edge& edge, z3::solver& solver) const;
    void leakage_rfx(NodeRef read, z3::solver& solver) const;
    void leakage_cox(NodeRef write, z3::solver& solver) const;
    void leakage_frx(NodeRef write, z3::solver& solver) const;
    unsigned leakage(z3::solver& solver) const;
    
    struct CondNode {
        NodeRef ref;
        z3::expr cond;
    };
    template <typename OutputIt>
    void find_sourced_memops(Inst::Kind kind, NodeRef org, OutputIt out) const;
    template <typename OutputIt>
    void find_preceding_memops(Inst::Kind kind, NodeRef write, OutputIt out) const;
    
    using OptionalNodeExprMap = std::unordered_map<NodeRef, std::optional<z3::expr>>;
    
    template <typename OutputIt>
    void find_sourced_xsaccesses(XSAccess kind, NodeRef org, OutputIt out) const;
    std::optional<z3::expr> find_sourced_xsaccesses_po(XSAccess kind, NodeRef org, NodeRef ref,
                                                       OptionalNodeExprMap& yesses,
                                                       OptionalNodeExprMap& nos) const;
    std::optional<z3::expr> find_sourced_xsaccesses_tfo(XSAccess kind, NodeRef org, NodeRef ref,
                                                        unsigned spec_depth,
                                                        OptionalNodeExprMap& nos_po,
                                                        OptionalNodeExprMap& yesses_tfo,
                                                        OptionalNodeExprMap& nos_tfo) const;
    
    template <typename UnaryOp>
    void transform_constants(UnaryOp op);
    
    template <typename UnaryOp>
    void transform_constraints(UnaryOp op);
    
    template <typename UnaryOp>
    void transform(UnaryOp op) {
        transform_constants(op);
        transform_constraints(op);
    }
    
    void replace(const z3::expr& from, const z3::expr& to);
    
    
    
public:
    using NodeRange = util::RangeContainer<NodeRef>;
    NodeRange node_range() const {
        return NodeRange {NodeRef {entry}, NodeRef {static_cast<unsigned>(nodes.size())}};
    }
    
    template <typename AEG_, typename Node_>
    class NodeIterator_Base {
    public:
        NodeIterator_Base() {}
        NodeIterator_Base(AEG_ *aeg, NodeRef ref): aeg(aeg), ref(ref) {}
        
        struct value_type {
            const NodeRef ref;
            Node_& node;
        };
        
        value_type operator*() const {
            return value_type {ref, aeg->lookup(ref)};
        }
        
        NodeIterator_Base& operator++() {
            ++ref;
            return *this;
        }
        
        NodeIterator_Base& operator++(int) {
            return ++*this;
        }
        
        bool operator==(const NodeIterator_Base& other) const {
            if (aeg != other.aeg) {
                throw std::logic_error("comparing node iterators from two different AEGs");
            }
            return ref == other.ref;
        }
        
        bool operator!=(const NodeIterator_Base& other) const {
            return !(*this == other);
        }
        
    private:
        AEG_ * const aeg = nullptr;
        NodeRef ref;
    };
    using NodeIterator = NodeIterator_Base<AEG, Node>;
    using ConstNodeIterator = NodeIterator_Base<const AEG, const Node>;
    
    auto node_range2() {
        return llvm::iterator_range<NodeIterator> {
            NodeIterator{this, 0UL},
            NodeIterator{this, size()}
        };
    }
    
    auto node_range2() const {
        return llvm::iterator_range<ConstNodeIterator> {
            ConstNodeIterator{this, 0UL},
            ConstNodeIterator{this, size()}
        };
    }
    
private:
    void find_upstream_def(NodeRef node, const llvm::Value *addr_ref,
                           std::unordered_set<NodeRef>& out) const;
    
    NodeRef add_node(const Node& node);
    
    template <typename OutputIt>
    void find_comx_window(NodeRef ref, unsigned distance, unsigned spec_depth, OutputIt out) const;
    
    void add_unidir_edge(NodeRef src, NodeRef dst, const UHBEdge& e) {
        if (e.possible()) {
            graph.insert(src, dst, e);
            ++nedges;
        }
    }
    
    void add_bidir_edge(NodeRef a, NodeRef b, const UHBEdge& e);
    void add_optional_edge(NodeRef src, NodeRef dst, const UHBEdge& e, const std::string& name = "");
    
    template <typename OutputIt>
    OutputIt get_edges(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind);
    
    using EdgePtrVec = std::vector<Edge *>;
    EdgePtrVec get_edges(Direction dir, NodeRef ref, Edge::Kind kind);
    
    template <typename OutputIt>
    OutputIt get_nodes(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) const;
    
    std::vector<std::pair<NodeRef, z3::expr>> get_nodes(Direction dir, NodeRef ref, Edge::Kind kind) const;
    
    void output_execution(std::ostream& os, const z3::model& model) const;
    void output_execution(const std::string& path, const z3::model& model) const;
    
    z3::expr check_no_intervening_writes(NodeRef src, NodeRef dst) const;
    // z3::expr check_no_intervening_writes_rec(NodeRef src, NodeRef
    
    bool is_ancestor(NodeRef parent, NodeRef child) const;
    bool is_ancestor_a(NodeRef, NodeRef) const;
    bool is_ancestor_b(NodeRef, NodeRef) const;
    
    Edge *find_edge(NodeRef src, NodeRef dst, Edge::Kind kind);
    const Edge *find_edge(NodeRef src, NodeRef dst, Edge::Kind kind) const;
    z3::expr edge_exists(NodeRef src, NodeRef dst, Edge::Kind kind) const;
    z3::expr total_order(NodeRef src, NodeRef dst, Edge::Kind kind) const;
    
    template <typename Pred>
    z3::expr cyclic(Pred pred) const;
    
    template <typename Pred>
    z3::expr acyclic(Pred pred) const { return !cyclic(pred); }
    
    // acyclicity check using integers
    template <typename Pred>
    z3::expr acyclic_int(Pred pred);
    
    bool is_pseudoedge(Edge::Kind kind) const {
        switch (kind) {
            case Edge::Kind::CO:
                return true;
            default:
                return false;
        }
    }
    
public:
    template <typename OutputIt, typename Pred>
    OutputIt get_nodes_if(OutputIt out, Pred pred) const {
        for (NodeRef ref : node_range()) {
            if (pred(lookup(ref))) {
                *out++ = ref;
            }
        }
        return out;
    }
    
    template <typename Container, typename OutputIt>
    OutputIt get_nodes(OutputIt out, const Container& container) const {
        return get_nodes_if(out, [&] (const Node& node) -> bool {
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
    
};


template <typename Pred>
AEG::ClosureMap AEG::find_predecessors(Pred f) const {
    ClosureMap map;
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    for (const NodeRef ref : order) {
        const auto& preds = po.po.rev.at(ref);
        NodeRefSet acc;
        for (const NodeRef pred : preds) {
            const auto& pred_set = map.at(pred);
            acc.insert(pred_set.begin(), pred_set.end());
        }
        
        if (f(ref)) {
            acc.insert(ref);
        }
        
        map.emplace(ref, acc);
    }
    
    // erase same
    for (auto& pair : map) {
        pair.second.erase(pair.first);
    }
    
    // erase no aliases
    
    
    return map;
}

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
    Progress progress;
    /* Have a set that is ordered based on the canonical xswrite order.
     * xsread is ordered before same-value xswrite.
     * xswrite with lower exec_order is ordered before same-value xswrite.
     */
    
    struct Access {
        z3::expr xsaccess_order;
        std::optional<NodeRef> tiebreaker;
    };
    
    const auto less = [&] (const Access& a, const Access& b) -> bool {
        const z3::expr xsaccess_order_diff = a.xsaccess_order - b.xsaccess_order;
        if (model.eval(xsaccess_order_diff < 0).is_true()) {
            return true;
        } else if (model.eval(xsaccess_order_diff > 0).is_true()) {
            return false;
        } else {
            if (!a.tiebreaker && !b.tiebreaker) {
                return false;
            } else if (!a.tiebreaker && b.tiebreaker) {
                return true;
            } else if (a.tiebreaker && !b.tiebreaker) {
                return false;
            } else {
                return *a.tiebreaker < *b.tiebreaker;
            }
        }
    };
    
    // get xsreads, xswrites
    // TODO: use canoncialized xswrite order. Right now it doesn't order it correctly.
    progress = Progress(size(), "xsaccesses");
    std::map<Access, NodeRef, decltype(less)> reads {less}, writes {less};
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (model.eval(node.exec() && node.xsread).is_true()) {
            [[maybe_unused]] const auto res = reads.emplace(Access {node.xsread_order, ref}, ref);
            assert(res.second);
        }
        if (model.eval(node.exec() && node.xswrite).is_true()) {
            [[maybe_unused]] const auto res = writes.emplace(Access {node.xswrite_order, ref}, ref);
            assert(res.second);
        }
        ++progress;
    }
    progress.done();
    
    assert(!reads.empty());
    assert(!writes.empty());
    
    const auto same_xstate = [&] (NodeRef a, NodeRef b) -> bool {
        return model.eval(lookup(a).same_xstate(lookup(b)));
    };
    
    // rfx
    progress = Progress(reads.size(), "rfx");
    for (const auto& read : reads) {
        if (exits.find(read.second) == exits.end()) {
            // not an exit
            auto write_it = writes.lower_bound(read.first);
            do {
                assert(write_it != writes.begin());
                --write_it;
            } while (!same_xstate(read.second, write_it->second));
            *out++ = std::make_tuple(write_it->second, read.second, Edge::RFX);
        } else {
            // is an exit
            for (auto it1 = writes.begin(); it1 != writes.end(); ++it1) {
                bool valid = true;
                for (auto it2 = std::next(it1); it2 != writes.end(); ++it2) {
                    if (same_xstate(it1->second, it2->second)) {
                        valid = false;
                    }
                }
                if (valid) {
                    *out++ = std::make_tuple(it1->second, read.second, Edge::RFX);
                }
            }
        }
        ++progress;
    }
    progress.done();
    
    
    // cox
    progress = Progress(writes.size() * (writes.size() - 1), "cox");
    for (auto it1 = writes.begin(); it1 != writes.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != writes.end(); ++it2) {
            if (model.eval(cox_exists(it1->second, it2->second)).is_true()) {
                *out++ = std::make_tuple(it1->second, it2->second, Edge::COX);
            }
            ++progress;
        }
    }
    progress.done();
    
    // frx
    progress = Progress(reads.size(), "frx");
    for (auto read_it = reads.begin(); read_it != reads.end(); ++read_it) {
        const Node& read = lookup(read_it->second);
        const auto write_begin = writes.upper_bound({read.xsread_order, std::nullopt});
        for (auto write_it = write_begin; write_it != writes.end(); ++write_it) {
            if (same_xstate(read_it->second, write_it->second)) {
                *out++ = std::make_tuple(read_it->second, write_it->second, Edge::FRX);
            }
        }
        ++progress;
    }
    progress.done();
    
    return out;
}


template <typename OutputIt>
OutputIt AEG::get_concrete_com(const z3::model& model, OutputIt out) const {
    // get po execution stream
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    NodeRefVec exec_stream;
    std::copy_if(order.begin(), order.end(), std::back_inserter(exec_stream), [&] (NodeRef ref) -> bool {
        return model.eval(lookup(ref).arch).is_true();
    });
    
    const auto same_addr = [&] (const Node& a, const Node& b) -> bool {
        return model.eval(a.same_addr(b)).is_true();
    };
    
    // rf
    for (auto read_it = exec_stream.rbegin(); read_it != exec_stream.rend(); ++read_it) {
        const Node& read = lookup(*read_it);
        if (!read.is_read()) { continue; }
        for (auto write_it = std::next(read_it); write_it != exec_stream.rend(); ++write_it) {
            const Node& write = lookup(*write_it);
            if (!write.is_write()) { continue; }
            if (same_addr(read, write)) {
                *out++ = std::make_tuple(*write_it, *read_it, Edge::RF);
                break;
            }
        }
    }
    
    // co
    for (auto it1 = exec_stream.begin(); it1 != exec_stream.end(); ++it1) {
        const Node& n1 = lookup(*it1);
        if (!n1.is_write()) { continue; }
        for (auto it2 = std::next(it1); it2 != exec_stream.end(); ++it2) {
            const Node& n2 = lookup(*it2);
            if (!n2.is_write()) { continue; }
            if (model.eval(n1.same_addr(n2)).is_false()) { continue; }
            *out++ = std::make_tuple(*it1, *it2, Edge::CO);
        }
    }
    
    // fr
    for (auto read_it = exec_stream.begin(); read_it != exec_stream.end(); ++read_it) {
        const Node& read = lookup(*read_it);
        if (!read.is_read()) { continue; }
        for (auto write_it = std::next(read_it); write_it != exec_stream.end(); ++write_it) {
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
