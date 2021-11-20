#pragma once

#include "leakage.h"

namespace lkg {

class SpectreV4_Detector: public Detector {
public:
    SpectreV4_Detector(aeg::AEG& aeg, Solver& solver): Detector(aeg, solver) {}
private:
    virtual DepVec deps() const override final;
    virtual void run_postdeps(const NodeRefVec& vec, CheckMode mode) override;
    
    virtual void run_() override final;
    virtual void run_transmitter(NodeRef transmitter, CheckMode mode) override;
    void run_bypassed_store(NodeRef load, const NodeRefVec& vec, CheckMode mode);
    void run_sourced_store(NodeRef load, NodeRef bypassed_store, const NodeRefVec& vec, CheckMode mode);
    virtual std::string name() const override final { return "SpectreV4"; }
};

}
