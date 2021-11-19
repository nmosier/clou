#pragma once

#include <functional>
#include <vector>
#include <iostream>

#include <z3++.h>

#include "config.h"
#include "util/output.h"
#include "cfg/block.h"
#include "util/z3.h"
#include "util/scope.h"
#include "aeg/aeg.h"

namespace lkg {

struct Leakage {
    NodeRefVec vec;
    NodeRef transmitter;
    
    void print_short(std::ostream& os) const;
    void print_long(std::ostream& os, const aeg::AEG& aeg) const;
};


class Detector {
public:
    using Solver = aeg::AEG::Solver;
    
    void run();
    virtual void run_() = 0;
    virtual ~Detector() {}
    
    const auto& get_transmitters() const { return transmitters; }
    
protected:
    std::unordered_set<const llvm::Instruction *> transmitters;
    struct next_transmitter {};
    struct lookahead_found {};
    
    bool lookahead(std::function<void ()> thunk);
    
    virtual std::string name() const = 0;
    
    void assert_edge(NodeRef src, NodeRef dst, const z3::expr& edge, aeg::Edge::Kind kind);
    bool check_edge(NodeRef src, NodeRef dst) const {
        return exec_window.contains(src) && exec_window.contains(dst);
    }
    
    struct EdgeRef {
        NodeRef src;
        NodeRef dst;
        aeg::Edge::Kind kind;
    };
    using EdgeVec = std::vector<EdgeRef>;
    using Mems = std::unordered_map<NodeRef, z3::expr>;
    
    aeg::AEG& aeg;
    Solver& solver;
    z3::solver alias_solver;

    using Actions = std::vector<std::string>;
    using PushAction = util::push_scope<Actions>;
    Actions actions;
    z3::expr init_mem;
    Mems mems;
    NodeRefSet exec_window, exec_notwindow;
    NodeRefSet trans_window, trans_notwindow;
    
    z3::expr mem(NodeRef ref) const;
    
    EdgeVec flag_edges;
    
    Detector(aeg::AEG& aeg, Solver& solver);
    
    z3::context& ctx();
    
    /* UTILITIES FOR SUBCLASSES */
    
    /** Trace back load */
    enum class CheckMode { FAST, SLOW };
    
    void traceback(NodeRef load, std::function<void (NodeRef, CheckMode)> func, CheckMode mode);
    void traceback_rf(NodeRef load, std::function<void (NodeRef, CheckMode)> func, CheckMode mode);
    void traceback_edge(aeg::Edge::Kind kind, NodeRef ref, std::function<void (NodeRef, CheckMode)> func, CheckMode mode);
    
    void for_each_transmitter(aeg::Edge::Kind kind, std::function<void (NodeRef, CheckMode)> func);
    template <class OutputIt>
    OutputIt for_new_transmitter(NodeRef transmitter, std::function<void (NodeRef, CheckMode)> func, OutputIt out);
    void for_one_transmitter(NodeRef transmitter, std::function<void (NodeRef, CheckMode)> func);
    
    auto push_edge(const EdgeRef& edge) {
        return util::push(flag_edges, edge);
    }
    
    void precompute_rf(NodeRef load);
    
    using Sources = std::unordered_map<NodeRef, z3::expr>;
    using RF = std::unordered_map<NodeRef, Sources>;
    const Sources& rf_sources(NodeRef load);
    void rf_sources(NodeRef load, Sources&& sources);
        
    // TODO: this is protected for now
    unsigned traceback_depth = 0;
    bool lookahead_tmp = true;
    
    void output_execution(const Leakage& leak);
    
private:
    CFGOrder partial_order;
    std::vector<std::pair<Leakage, std::string>> leaks;
    RF rf; // TODO: remove?

    Mems get_mems();
    Mems get_mems(const NodeRefSet& set); /*!< only consider nodes in \p set */
    Mems get_mems1(const NodeRefSet& set); /*!< this uses topological order */
    
    
    friend class Detector_;
};


namespace dbg {

template <typename Solver>
void append_core(const Solver& solver) {
    if (const char *corepath = std::getenv("CORES")) {
        std::ofstream ofs {corepath, std::ios::app};
        ofs << __FILE__ << ":" << __LINE__ << ": " << solver.unsat_core() << "\n";
    }
}

}

}


#define z3_cond_scope \
const std::optional<z3::scope<std::remove_reference<decltype(solver)>::type>> scope = (mode == CheckMode::SLOW) ? std::make_optional(z3::scope<std::remove_reference<decltype(solver)>::type>(solver)) : std::optional<z3::scope<std::remove_reference<decltype(solver)>::type>>()
