#pragma once

#include <string>
#include <cassert>
#include <unordered_set>
#include <sstream>
#include <set>

#include <llvm/IR/Instruction.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <z3++.h>

#include "util/fwd/z3.h"
#include "graph.h"
#include "noderef.h"
#include "util/progress.h"
#include "util/iterator.h"
#include "util/output.h"
#include "aeg/fwd.h"
#include "aeg/context.h"
#include "aeg/edge.h"
#include "aeg/node.h"
#include "inst.h"
#include "util/timer.h"

class CFG_Expanded;

namespace lkg {
class DetectorJob;
}

namespace aeg {

class AEG {
public:
    const CFG_Expanded& po; /*!<  The input CFG. The AEG constructs nodes in a 1:1 correspondence and heavily uses the preds/succs relations of this CFG. */
    llvm::AliasAnalysis& AA;
    Context context; /*!<  The context for AEG construction. */

  using Node = aeg::Node;
  using Edge = aeg::Edge;
    using graph_type = Graph<NodeRef, Edge, std::hash<NodeRef>, Edge::Hash>;
    
    NodeRef entry; /*!< The unique entry node of the AEG (node has type Inst::Kind::ENTRY) */
    NodeRefSet exits; /*!< The set of exit nodes of the AEG (nodes have type Inst::Kind::EXIT) */
    
    NodeRef exit_con(const z3::eval& eval) const;
    
    graph_type graph;
    
    /** Construct the AEG.
     * \param AA alias analysis results to use
     * \param rob_size reorder buffer size (used to limit comx)
     */
    void construct(llvm::AliasAnalysis& AA, unsigned rob_size);
    
    const Node& lookup(NodeRef ref) const { return nodes.at(static_cast<unsigned>(ref)); }
    Node& lookup(NodeRef ref) { return nodes.at(static_cast<unsigned>(ref)); }
    
    explicit AEG(const CFG_Expanded& po, llvm::AliasAnalysis& aa): po(po), AA(aa), context(), constraints() {}
    
    void dump_graph(std::ostream& os) const;
    void dump_graph(const std::string& path) const;
    
    void simplify();
    
    void test(TransmitterOutputIt out);
    
    std::string function_name() const;
    
    std::size_t size() const { return nodes.size(); } /*!< Get the number of nodes in the graph */
    
    template <typename Function> void for_each_edge(Function f) { graph.for_each_edge(f); }
    template <typename Function> void for_each_edge(Function f) const { graph.for_each_edge(f); }
    template <typename Function> void for_each_edge(Edge::Kind kind, Function f);
    template <typename Function> void for_each_edge(Edge::Kind kind, Function f) const;
    
    template <typename Function> void for_each_node(Inst::Kind kind, Function f);
    template <typename Function> void for_each_node(Inst::Kind kind, Function f) const;
    
    const Context& ctx() const { return context; }
   
private:
    Constraints constraints; /*!< global constraints (i.e. can't be attributed to one particular node or edge) */
    std::vector<Node> nodes;
    unsigned nedges = 0;
    
    void construct_nodes();
    void construct_arch();
    void construct_tfo();
    void construct_addr_defs();
    void construct_addr_refs();
    void construct_addrs();
    void construct_aliases(llvm::AliasAnalysis& AA);
    void construct_com();
    void construct_xsaccess_order(const NodeRefSet& xsaccesses);
    void construct_mem();
    void construct_comx();
    void construct_addr();
    void construct_addr_gep();
    void construct_data();
    void construct_ctrl();
    void construct_ctrl2();
    void construct_taint();
    
    void constrain_arch();
    void constrain_exec();
    void constrain_trans();
    void constrain_tfo();
    void constrain_comx();
    
    void constrain_arch(const NodeRefSet& window, z3::solver& solver);
    void constrain_exec(const NodeRefSet& window, z3::solver& solver);
    void constrain_trans(const NodeRefSet& window, z3::solver& solver);
    void constrain_tfo(const NodeRefSet& window, z3::solver& solver);
    void constrain_comx(const NodeRefSet& window, z3::solver& solver);
    
    template <typename Func>
    void for_each_dependency(NodeRef ref, const llvm::Value *V, Func func);
    
    const llvm::Module *get_module() const {
        const auto it = std::find_if(nodes.begin(), nodes.end(), [] (const Node& node) -> bool {
            return node.inst->get_inst() != nullptr;
        });
        assert(it != nodes.end());
        return it->inst->get_inst()->getParent()->getParent()->getParent();
    }
    

private:
    z3::solver make_solver();

public:
    std::vector<std::pair<z3::expr, std::string>> assert_xsaccess_order(const NodeRefSet& window);

    void for_each_pred_in_window(NodeRef ref, unsigned window, std::function<void (NodeRef)> is, std::function<void (NodeRef)> isnt);

    bool may_source_stb(NodeRef load, NodeRef store) const;
    
private:
    void compute_min_store_paths();
    
    NodeRefSet spectrev4_siblings(NodeRef ref) const;
    
    // TODO: unify this, so that it just returns a NodeRefMap for in, out.
    using DependencyMap = NodeRefMap;
    DependencyMap dependencies;
    void construct_dependencies();
    DependencyMap construct_dependencies2();
    
    using DominatorMap = NodeRefMap;
    
    template <Direction dir> DominatorMap construct_dominators_shared2() const;
    template <Direction dir> DominatorMap construct_dominators_shared(const NodeRefSet& window) const;
public:
    DominatorMap dominators; /*!< Maps nodes to their dominators (which are always ancestors)*/
    DominatorMap postdominators; /*!< Maps nodes to their postdominators (which are always grandchildren) */
    DominatorMap control_equivalents; /*!< Maps nodes to their control-equivalent ancestors */
private:
    void construct_dominators();
    void construct_postdominators();
#if 0
    void construct_control_equivalents();
#endif
    
    /** Check for leakage in the AEG. Outputs set of transmitter gadgets. */
    void leakage(z3::solver& solver, TransmitterOutputIt out);
    
    using EdgeVec = std::vector<std::tuple<NodeRef, NodeRef, aeg::Edge::Kind>>;
    
public:
    using NodeRange = util::RangeContainer<NodeRef>;
    NodeRange node_range() const {
        return NodeRange {NodeRef {entry}, NodeRef {static_cast<unsigned>(nodes.size())}};
    }
    
private:
    NodeRef add_node(Node&& node); /*!< Add node to the AEG and return its corresponding node ref. */
    
    void add_unidir_edge(NodeRef src, NodeRef dst, const Edge& e); /*!< Add a directed edge from \p src to \p dst with edge \p e */
    void add_bidir_edge(NodeRef a, NodeRef b, const Edge& e); /*!< Add a directed edge in each direction between nodes \p a and \p b with edge \p e */
    /** Add a directed edge from \p src to \p dst and make the edge \p e optional with an additional unconstrained boolean variable
     * @param name The base name of the unconstrained boolean variable
     */
    z3::expr add_optional_edge(NodeRef src, NodeRef dst, const Edge& e, const std::string& name);
    
    template <typename OutputIt>
    OutputIt get_edges(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind);
    
    using EdgePtrVec = std::vector<Edge *>;
    EdgePtrVec get_edges(Direction dir, NodeRef ref, Edge::Kind kind);
    
public:
    template <typename OutputIt>
    OutputIt get_nodes(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) const;
    
    std::vector<std::pair<NodeRef, z3::expr>> get_nodes(Direction dir, NodeRef ref, Edge::Kind kind) const;

public:
    void output_execution(std::ostream& os, const z3::eval& eval, const EdgeVec& flag_edges = EdgeVec());
    void output_execution(const std::string& path, const z3::eval& eval, const EdgeVec& flag_edges = EdgeVec());
    
private:
    struct Execution;
    Execution analyze_execution(const z3::eval& eval) const;
    
    Edge *find_edge(NodeRef src, NodeRef dst, Edge::Kind kind);
    const Edge *find_edge(NodeRef src, NodeRef dst, Edge::Kind kind) const;
    
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
            return container.find(node.inst->kind()) != container.end();
        });
    }
    
    template <typename OutputIt>
    OutputIt get_nodes(OutputIt out, Inst::Kind kind) {
        return get_nodes(out, std::unordered_set<Inst::Kind> {kind});
    }
    
    z3::expr exists(Edge::Kind kind, NodeRef src, NodeRef dst);
    z3::expr exists_src(Edge::Kind kind, NodeRef src) const;
    z3::expr exists_dst(Edge::Kind kind, NodeRef dst) const;
    z3::expr co_exists(NodeRef src, NodeRef dst);
    z3::expr rf_exists(NodeRef src, NodeRef dst);
    z3::expr fr_exists(NodeRef src, NodeRef dst);
    z3::expr rfx_exists(NodeRef src, NodeRef dst) const;
    z3::expr cox_exists(NodeRef src, NodeRef dst) const;
    z3::expr frx_exists(NodeRef src, NodeRef dst) const;
    
    using ValueLoc = std::pair<CFG::ID, const llvm::Value *>;
private:
    z3::expr com_exists_precond(NodeRef src, NodeRef dst, Access src_kind, Access dst_kind) const;
    z3::expr comx_exists_precond(NodeRef src, NodeRef dst, XSAccessType src_kind, XSAccessType dst_kind) const;

    using ValueLocRel = std::unordered_map<std::pair<ValueLoc, ValueLoc>, llvm::AliasResult>;
    ValueLocRel alias_rel;
    
public:
    llvm::AliasResult check_alias(NodeRef ref1, NodeRef ref2) const;
    llvm::AliasResult check_alias(const ValueLoc& vl1, const ValueLoc& vl2) const;
private:
    struct AddrInfo {
        CFG::ID id;
        const llvm::Value *V;
        z3::expr e;
        std::optional<NodeRef> ref;
        
        ValueLoc vl() const { return {id, V}; }
    };
    std::map<ValueLoc, AddrInfo> vl2addr;

    
    friend class lkg::DetectorJob;
    
    llvm::AliasResult compute_alias(const AddrInfo& a, const AddrInfo& b) const;
    llvm::AliasResult compute_alias(NodeRef a, NodeRef b) const;
    
    void add_alias_result(const ValueLoc& vl1, const ValueLoc& vl2, llvm::AliasResult res);
    static bool compatible_types(const llvm::Type *P1, const llvm::Type *P2);
    static bool compatible_types_pointee(const llvm::Type *T1, const llvm::Type *T2);
    
    enum class AddressKind {
        UNKNOWN,
        STACK,
        CONST,
    };
    std::unordered_map<const llvm::Value *, AddressKind> addr_kinds;
    AddressKind get_addr_kind(const llvm::Value *V);
    
    ValueLoc get_value_loc(NodeRef ref) const;
};

inline bool operator<(const AEG::ValueLoc& a, const AEG::ValueLoc& b) {
    if (a.first == b.first) {
        return a.second < b.second;
    } else {
        return a.first < b.first;
    }
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

template <typename Function> void AEG::for_each_node(Inst::Kind kind, Function f) {
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        if (node.inst->kind() == kind) {
            f(ref, node);
        }
    }
}

template <typename Function> void AEG::for_each_node(Inst::Kind kind, Function f) const {
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (node.inst->kind() == kind) {
            f(ref, node);
        }
    }
}


struct AEG::Execution {
    NodeRefVec arch;
    NodeRefVec trans;
    NodeRef spec_gadget;
};








}
