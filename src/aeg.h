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

class AEG {
public:
    using Node = UHBNode;
    using NodeRef = std::size_t;
    using Edge = UHBEdge;
    using graph_type = Graph<NodeRef, Edge, std::hash<NodeRef>, Edge::Hash>;
    
    NodeRef entry;
    NodeRef exit;
    
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
    
    const UHBContext& ctx() const { return context; }
    
private:
    const AEGPO& po;
    UHBContext context;
    UHBConstraints constraints;
    std::vector<Node> nodes;
    
    unsigned num_specs() const { return po.num_specs; }
    
    using ClosureMap = std::unordered_map<NodeRef, std::unordered_set<NodeRef>>;
    template <typename Pred>
    ClosureMap find_predecessors(Pred f) const;
    
    void construct_exec();
    void construct_arch();
    void construct_trans();
    void construct_po();
    void construct_tfo();
    void construct_addr_defs();
    void construct_addr_refs();
    void construct_aliases(llvm::AliasAnalysis& AA);
    void construct_com();
    
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
        graph.insert(src, dst, e);
    }
    
    void add_bidir_edge(NodeRef a, NodeRef b, const UHBEdge& e);
    void add_optional_edge(NodeRef src, NodeRef dst, const UHBEdge& e, const std::string& name = "");
    
    template <typename OutputIt>
    OutputIt get_edges(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind);
    
    using EdgePtrVec = std::vector<Edge *>;
    EdgePtrVec get_edges(Direction dir, NodeRef ref, Edge::Kind kind);
    
    template <typename OutputIt>
    OutputIt get_nodes(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) const;
    
    NodeRefVec get_nodes(Direction dir, NodeRef ref, Edge::Kind kind) const;
    
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
    
    template <typename Pred>
    z3::expr cyclic(Pred pred) const;
    
    template <typename Pred>
    z3::expr acyclic(Pred pred) const { return !cyclic(pred); }
};


template <typename Pred>
AEG::ClosureMap AEG::find_predecessors(Pred f) const {
    ClosureMap map;
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    for (const NodeRef ref : order) {
        const auto& preds = po.po.rev.at(ref);
        std::unordered_set<NodeRef> acc;
        for (const NodeRef pred : preds) {
            const auto& pred_set = map.at(pred);
            acc.insert(pred_set.begin(), pred_set.end());
        }
        
        if (f(ref)) {
            acc.insert(ref);
        }
        
        map.emplace(ref, acc);
    }
    
    for (auto& pair : map) {
        pair.second.erase(pair.first);
    }
    
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


template <typename OutputIt>
OutputIt AEG::get_nodes(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) const {
    const auto& map = graph(dir);
    for (const auto& p : map.at(ref)) {
        for (const auto& edge : p.second) {
            if (edge->kind == kind) {
                *out++ = p.first;
                break;
            }
        }
    }
    return out;
}
