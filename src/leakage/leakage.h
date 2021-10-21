#pragma once

#include <functional>
#include <vector>
#include <iostream>

#include <z3++.h>

// TODO: shouldn't need to include "aeg.h"
#include "aeg/aeg.h"

template <typename Derived>
struct Leakage {
    NodeRefVec vec() const { std::abort(); }
    void print_short(std::ostream& os) const;
    void print_long(std::ostream& os, const aeg::AEG& aeg) const;
    NodeRef get_transmitter() const { std::abort(); }
};

class LeakageDetector {
public:
    virtual void run() = 0;
    virtual ~LeakageDetector() {}
    
protected:
    struct EdgeRef {
        NodeRef src;
        NodeRef dst;
        aeg::Edge::Kind kind;
    };
    using EdgeVec = std::vector<EdgeRef>;
    using Mems = std::unordered_map<NodeRef, z3::expr>;
    
    aeg::AEG& aeg;
    z3::solver& solver;
    Mems mems;
    EdgeVec flag_edges;
    
    LeakageDetector(aeg::AEG& aeg, z3::solver& solver): aeg(aeg), solver(solver), mems(get_mems()) {}
    
    z3::context& ctx() { return aeg.context.context; }
    
    /* UTILITIES FOR SUBCLASSES */
    
    /** Trace back load */
    void traceback(NodeRef load, std::function<void (NodeRef)> func);
    
    void for_each_transmitter(aeg::Edge::Kind kind, std::function<void (NodeRef, NodeRef)> func) const;
    
private:
    Mems get_mems();
};

template <typename Leakage>
class LeakageDetector_: public LeakageDetector {
public:
    using leakage_type = Leakage;
    
    virtual void run() override final;
    
protected:
    using LeakVec = std::vector<Leakage>;
    using OutputIt = std::back_insert_iterator<LeakVec>;
    
    virtual std::string name() const = 0;
    virtual void run(OutputIt out) = 0;

    void output_execution(const Leakage& leak);
    
    LeakageDetector_(aeg::AEG& aeg, z3::solver& solver): LeakageDetector(aeg, solver) {}
    
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

class SpectreV1_Detector: public LeakageDetector_<SpectreV1_Leakage> {
public:
    
protected:
    using DepVec = std::vector<aeg::Edge::Kind>;
    virtual DepVec deps() const = 0;
    virtual aeg::Edge::Kind cur_dep() const { return *(deps().rbegin() + loads.size()); }

    SpectreV1_Detector(aeg::AEG& aeg, z3::solver& solver): LeakageDetector_(aeg, solver) {}

private:
    NodeRefVec loads;
    virtual void run(OutputIt out) override final;

    void run1(OutputIt& out, NodeRef transmitter, NodeRef access);
    virtual std::string name() const override final { return "spectrev1"; }
};

class SpectreV1_Classic_Detector final: public SpectreV1_Detector {
public:
    SpectreV1_Classic_Detector(aeg::AEG& aeg, z3::solver& solver): SpectreV1_Detector(aeg, solver) {}
    
private:
    virtual DepVec deps() const override final;
};

class SpectreV1_Control_Detector final: public SpectreV1_Detector {
public:
    SpectreV1_Control_Detector(aeg::AEG& aeg, z3::solver& solver): SpectreV1_Detector(aeg, solver) {}
private:
    virtual DepVec deps() const override final;
};


/* IMPLEMENTATIONS */

template <typename Leakage>
void LeakageDetector_<Leakage>::run() {
    run(std::back_inserter(leaks));
    
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
void LeakageDetector_<Leakage>::output_execution(const Leakage& leak) {
    std::stringstream ss;
    ss << output_dir << "/" << name();
    for (const NodeRef ref : leak.vec()) {
        ss << "-" << ref;
    }
    ss << ".dot";
    const std::string path = ss.str();
    assert(solver.check() == z3::sat);
    const z3::eval eval {solver.get_model()};
    
    // TODO: shouldn't need to do this
    std::vector<std::tuple<NodeRef, NodeRef, aeg::Edge::Kind>> flag_edges_;
    std::transform(flag_edges.begin(), flag_edges.end(), std::back_inserter(flag_edges_), [] (const EdgeRef& e) {
        return std::make_tuple(e.src, e.dst, e.kind);
    });
    
    aeg.output_execution(path, eval, flag_edges_);
}
