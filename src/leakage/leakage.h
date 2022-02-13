#pragma once

#include <functional>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <mutex>
#include <variant>

#include <z3++.h>

#include "config.h"
#include "util/output.h"
#include "cfg/block.h"
#include "util/z3.h"
#include "util/scope.h"
#include "util/timer.h"
#include "util/algorithm.h"
#include "noderef.h"

namespace lkg {

struct Leakage {
    NodeRefVec vec;
    NodeRef transmitter;
    
    void print_short(std::ostream& os) const;
    void print_long(std::ostream& os, const aeg::AEG& aeg) const;
};


class DetectorJob {
    friend class DetectorMain;
protected:
    std::unique_lock<std::mutex> lock;
protected:
    enum class CheckMode { FAST, SLOW };
    using DepVec = std::vector<std::pair<aeg::Edge::Kind, aeg::ExecMode>>;

    struct next_transmitter {};
    struct lookahead_found {};
    
    struct EdgeRef {
        NodeRef src;
        NodeRef dst;
        aeg::Edge::Kind kind;
    };
    using EdgeVec = std::vector<EdgeRef>;
    using Mems = std::unordered_map<NodeRef, z3::expr>;
    
    struct Action {
        NodeRef src;
        NodeRef dst;
        aeg::Edge::Kind edge;
    };
    friend std::ostream& operator<<(std::ostream&, const Action&);

    using Actions = std::vector<Action>;
    using PushAction = util::push_scope<Actions>;
    
    using Deps = std::vector<aeg::Edge::Kind>;
    using DepIt = DepVec::const_reverse_iterator;

public:
    struct CheckStats {
        unsigned sat = 0;
        unsigned unsat = 0;
        unsigned unknown = 0;
        unsigned total() const { return sat + unsat + unknown; }
    };
    
    void run();
    virtual ~DetectorJob();
    
    const auto& get_transmitters() const { return transmitters; }
    
protected:
    NodeRef candidate_transmitter;
    CheckStats check_stats;
    std::unordered_set<const llvm::Instruction *> transmitters;
    aeg::AEG& aeg;
    z3::context& local_ctx;
    z3::solver solver;
    z3::solver alias_solver;
    Actions actions;
    z3::expr init_mem;
    Mems mems;
    NodeRefSet exec_window;
    NodeRefSet exec_notwindow;
    NodeRefSet trans_window;
    NodeRefSet trans_notwindow;
    EdgeVec flag_edges;
    
    DetectorJob(aeg::AEG& aeg, z3::context& local_ctx, z3::solver& solver, NodeRef candidate_transmitter, std::vector<std::pair<Leakage, std::string>>& leaks);
    
    /* SUBCLASS INTERFACE */
    virtual std::string name() const = 0;
    virtual std::optional<float> get_timeout() const { return std::nullopt; } // no timeout by default
    virtual void set_timeout(z3::check_result check_res, float secs) {} // no timeout by default, so nothing to update
    virtual void run_transmitter(NodeRef transmitter, CheckMode mode) = 0;
    virtual void run_postdeps(const NodeRefVec& vec, CheckMode mode) = 0;
    virtual DepVec deps() const = 0;
    virtual void entry(NodeRef, CheckMode) = 0;
    
    /* UTILITIES FOR SUBCLASSES */
    bool lookahead(std::function<void ()> thunk);
    z3::check_result solver_check(bool allow_unknown = true);
    void assert_edge(NodeRef src, NodeRef dst, const z3::expr& edge, aeg::Edge::Kind kind);
    bool check_edge(NodeRef src, NodeRef dst) const {
        return exec_window.contains(src) && exec_window.contains(dst);
    }
    z3::context& ctx();

    void traceback(NodeRef load, aeg::ExecMode exec_mode, std::function<void (NodeRef, CheckMode)> func, CheckMode mode);
    void traceback_rf(NodeRef load, aeg::ExecMode exec_mode, std::function<void (NodeRef, CheckMode)> func, CheckMode mode);
    void traceback_edge(aeg::Edge::Kind kind, NodeRef ref, std::function<void (NodeRef, CheckMode)> func, CheckMode mode);
    
    void traceback_deps(NodeRef from_ref, std::function<void (const NodeRefVec&, CheckMode)> func,
                        CheckMode mode);
    
    void for_one_transmitter(NodeRef transmitter, std::function<void (NodeRef, CheckMode)> func, bool priv);
    
    auto push_edge(const EdgeRef& edge) {
        return util::push(flag_edges, edge);
    }
    
    NodeRefSet reachable_r(const NodeRefSet& window, NodeRef ref) const;
    std::unordered_map<NodeRef, z3::expr> precompute_rf_one(NodeRef load, const NodeRefSet& window);
    void precompute_rf(NodeRef load);
    
    using Sources = std::unordered_map<NodeRef, z3::expr>;
    using RF = std::unordered_map<NodeRef, Sources>;
    const Sources& rf_sources(NodeRef load);
    void rf_sources(NodeRef load, Sources&& sources);
        
    // TODO: this is protected for now
    unsigned traceback_depth = 0;
    bool lookahead_tmp = true;
    
    void output_execution(const Leakage& leak);
    
    void solver_add(const z3::expr& e) {
        solver.add(e);
    }
    
    void solver_add(const z3::expr& e, const std::string& s) {
        solver.add(e);
    }
    
    z3::eval solver_eval() const {
        return z3::eval(solver.get_model());
    }
    
    
private:
    std::vector<std::pair<Leakage, std::string>>& leaks;
    RF rf; // TODO: remove?
    
    Mems get_mems(const NodeRefSet& set); /*!< this uses topological order */
    
    void traceback_deps_rec(DepIt it, DepIt end, NodeRefVec& vec, NodeRef from_ref,
                            std::function<void (const NodeRefVec&, CheckMode)> func, CheckMode mode);

protected:
    z3::expr translate(const z3::expr& global_e) {
        Z3_ast local_e = Z3_translate(global_e.ctx(), global_e, local_ctx);
        return z3::expr(local_ctx, local_e);
    }
    
    z3::expr_vector translate(const z3::expr_vector& global_v) {
        Z3_ast_vector local_v = Z3_ast_vector_translate(global_v.ctx(), global_v, local_ctx);
        return z3::expr_vector(local_ctx, local_v);
    }
};

class DetectorMain {
public:
    DetectorMain(aeg::AEG& aeg, z3::solver& solver);
    
    template <class Job>
    void run();
    
    auto get_transmitters() const { return transmitters; }
    
protected:
    template <class Job>
    void get_candidate_transmitters(NodeRefSet& candidate_transmitters) const;

private:
    aeg::AEG& aeg;
    z3::solver& solver;
    std::vector<std::pair<Leakage, std::string>> leaks;
    std::unordered_set<const llvm::Instruction *> transmitters;
    
    z3::context& ctx() const;
    std::mutex& mutex() const;
    
    void dump();
    
    struct ParallelContext {
        z3::context ctx;
        z3::solver solver;
        
        ParallelContext(): ctx(), solver(ctx) {}
    };
    using ParallelContextVec = std::vector<ParallelContext>;
    
    ParallelContextVec create_solvers(z3::solver& from_solver, unsigned N) const;
};

std::ostream& operator<<(std::ostream& os, const DetectorJob::Action& action);

namespace dbg {

template <typename Solver>
void append_core(const Solver& solver, const std::string& label) {
    if (const char *corepath = std::getenv("CORES")) {
        std::ofstream ofs {corepath, std::ios::app};
        ofs << __FILE__ << ":" << __LINE__ << ": " << label << ": " << solver.unsat_core() << "\n";
    }
}

}

}


#define z3_cond_scope \
lock.unlock(); \
const std::optional<z3::scope> scope = (mode == CheckMode::SLOW) ? std::make_optional(z3::scope(solver)) : std::optional<z3::scope>(); \
lock.lock()
