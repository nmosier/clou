#pragma once

#include "leakage.h"

namespace lkg {

class SpectreV4_Detector: public Detector {
public:
    SpectreV4_Detector(aeg::AEG& aeg, Solver& solver): Detector(aeg, solver) {}
private:
    virtual DepVec deps() const override final { return DepVec {aeg::Edge::ADDR, aeg::Edge::ADDR_GEP}; }
    
    virtual void run_() override final;
    void run_transmitter(NodeRef transmitter, CheckMode mode);
    void run_bypassed_store(NodeRef load, const NodeRefVec& vec, CheckMode mode);
    void run_sourced_store(NodeRef load, NodeRef bypassed_store, const NodeRefVec& vec, CheckMode mode);
    virtual std::string name() const override final { return "SpectreV4"; }
};

}
