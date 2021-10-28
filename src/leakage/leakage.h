#pragma once

#include <functional>
#include <vector>
#include <iostream>

#include <z3++.h>

#include "config.h"
#include "util/output.h"

// TODO: shouldn't need to include "aeg.h"

namespace lkg {

template <typename Derived>
struct Leakage {
    NodeRefVec vec() const { std::abort(); }
    void print_short(std::ostream& os) const;
    void print_long(std::ostream& os, const aeg::AEG& aeg) const;
    NodeRef get_transmitter() const { std::abort(); }
};

class Detector {
public:
    virtual void run() = 0;
    virtual ~Detector() {
        std::cerr << rf_source_count << "\n";
    }
    
protected:
    struct next_transmitter {};
    
    struct EdgeRef {
        NodeRef src;
        NodeRef dst;
        aeg::Edge::Kind kind;
    };
    using EdgeVec = std::vector<EdgeRef>;
    using Mems = std::unordered_map<NodeRef, z3::expr>;
    
    aeg::AEG& aeg;
    z3::solver& solver;
#if 1
    Mems mems;
#endif
    Mems mems_arch, mems_trans;
    
    z3::expr mem(NodeRef ref) const {
#if 0
        const aeg::Node& node = aeg.lookup(ref);
        return z3::ite(node.arch, mems_arch.at(ref), mems_trans.at(ref));
#else
        return mems.at(ref);
#endif
    }
    
    EdgeVec flag_edges;
    
    Detector(aeg::AEG& aeg, z3::solver& solver): aeg(aeg), solver(solver),
#if 1
    mems(get_mems()),
#endif
    mems_arch(get_mems_arch()), mems_trans(get_mems_trans()), rf_solver(z3::duplicate(solver)) {}
    
    z3::context& ctx() { return aeg.context.context; }
    
    /* UTILITIES FOR SUBCLASSES */
    
    /** Trace back load */
    void traceback(NodeRef load, std::function<void (NodeRef)> func);
    void traceback_rf(NodeRef load, std::function<void (NodeRef)> func);
    
    void for_each_transmitter(aeg::Edge::Kind kind, std::function<void (NodeRef)> func) const;
    
    auto push_edge(const EdgeRef& edge) {
        return util::push(flag_edges, edge);
    }
    
    void precompute_rf(NodeRef load);
    
    using Sources = std::unordered_map<NodeRef, z3::expr>;
    using RF = std::unordered_map<NodeRef, Sources>;
    const Sources& rf_sources(NodeRef load);
    void rf_sources(NodeRef load, Sources&& sources);
    
private:
    unsigned traceback_depth = 0;
    
    Mems get_mems();
    
    Mems get_mems_arch();
    Mems get_mems_trans();
    
    RF rf;
    z3::solver rf_solver;
    
    // DEBUG members
    std::unordered_map<NodeRef, unsigned> rf_source_count;
};

template <typename Leakage>
class Detector_: public Detector {
public:
    using leakage_type = Leakage;
    
    virtual void run() override final;
    
protected:
    using LeakVec = std::vector<Leakage>;
    using OutputIt = std::back_insert_iterator<LeakVec>;
    
    virtual std::string name() const = 0;
    virtual void run_() = 0;

    void output_execution(const Leakage& leak);
    
    Detector_(aeg::AEG& aeg, z3::solver& solver): Detector(aeg, solver) {}
    
private:
    std::vector<Leakage> leaks;
};

struct SpectreV1_Leakage: Leakage<SpectreV1_Leakage> {
    NodeRef load0, load1, transmitter2;
    
    NodeRefVec vec() const {
        return {load0, load1, transmitter2};
    }
    
    NodeRef get_transmitter() const {
        return transmitter2;
    }
};

class SpectreV1_Detector: public Detector_<SpectreV1_Leakage> {
public:
    
protected:
    using DepVec = std::vector<aeg::Edge::Kind>;
    virtual DepVec deps() const = 0;
    virtual aeg::Edge::Kind cur_dep() const { return *(deps().rbegin() + loads.size()); }

    SpectreV1_Detector(aeg::AEG& aeg, z3::solver& solver): Detector_(aeg, solver) {}

private:
    NodeRefVec loads;
    virtual void run_() override final;

    void run1(NodeRef transmitter, NodeRef access);
};

class SpectreV1_Classic_Detector final: public SpectreV1_Detector {
public:
    SpectreV1_Classic_Detector(aeg::AEG& aeg, z3::solver& solver): SpectreV1_Detector(aeg, solver) {}
    
private:
    virtual DepVec deps() const override final;
    virtual std::string name() const override final { return "SpectreV1Classic"; }
};

class SpectreV1_Control_Detector final: public SpectreV1_Detector {
public:
    SpectreV1_Control_Detector(aeg::AEG& aeg, z3::solver& solver): SpectreV1_Detector(aeg, solver) {}
private:
    virtual DepVec deps() const override final;
    virtual std::string name() const override final { return "SpectreV1Control"; }
};

struct SpectreV4_Leakage: Leakage<SpectreV4_Leakage> {
    NodeRef transmitter;
    NodeRef bypassed_store;
    NodeRef sourced_store;
    NodeRef load;
    
    NodeRefVec vec() const {
        return {sourced_store, bypassed_store, load, transmitter};
    }
    
    NodeRef get_transmitter() const { return transmitter; }
};

class SpectreV4_Detector: public Detector_<SpectreV4_Leakage> {
public:
    SpectreV4_Detector(aeg::AEG& aeg, z3::solver& solver): Detector_(aeg, solver) {}
private:
    SpectreV4_Leakage leak;
    
    virtual void run_() override final;
    void run_load(NodeRef access); /*!< binds speculative load */
    void run_bypassed_store(); /*!< binds bypassed store */
    void run_sourced_store(); /*!< binds sourced stores */
    virtual std::string name() const override final { return "SpectreV4"; }
};

/* IMPLEMENTATIONS */

template <typename Leakage>
void Detector_<Leakage>::run() {
    run_();
    
    // open file
    const std::string path = util::to_string(output_dir, "/leakage.txt");
    std::ofstream ofs {path};
    
    // dump leakage
    for (const auto& leak : leaks) {
        leak.print_short(ofs);
        ofs << " --";
        leak.print_long(ofs, aeg);
        ofs << "\n";
    }
    
    // print out set of transmitters
    std::unordered_set<const llvm::Instruction *> transmitters;
    for (const auto& leak : leaks) {
        transmitters.insert(aeg.lookup(leak.get_transmitter()).inst->get_inst());
    }
    llvm::errs() << "transmitters:\n";
    for (const auto transmitter : transmitters) {
        llvm::errs() << *transmitter << "\n";
    }
}


template <typename Leakage>
void Detector_<Leakage>::output_execution(const Leakage& leak) {
    leaks.push_back(leak);
        
    std::stringstream ss;
    ss << output_dir << "/" << name();
    for (const NodeRef ref : leak.vec()) {
        ss << "-" << ref;
    }
    ss << ".dot";
    const std::string path = ss.str();
    assert(solver.check() == z3::sat);
    const z3::eval eval {solver.get_model()};
    const auto edge = push_edge(EdgeRef {
        .src = leak.get_transmitter(),
        .dst = aeg.exit_con(eval),
        .kind = aeg::Edge::RFX,
    });
    
    // TODO: shouldn't need to do this
    std::vector<std::tuple<NodeRef, NodeRef, aeg::Edge::Kind>> flag_edges_;
    std::transform(flag_edges.begin(), flag_edges.end(), std::back_inserter(flag_edges_), [] (const EdgeRef& e) {
        return std::make_tuple(e.src, e.dst, e.kind);
    });
    
    if (witness_executions) {
        aeg.output_execution(path, eval, flag_edges_);
    }
    
    if (fast_mode) {
        throw next_transmitter {};
    }
}

}
