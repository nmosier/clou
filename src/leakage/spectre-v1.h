#pragma once

#include "leakage.h"

namespace lkg {

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

    SpectreV1_Detector(aeg::AEG& aeg, Solver& solver): Detector_(aeg, solver) {}

private:
    NodeRefVec loads;
    virtual void run_() override final;

    void run1(NodeRef transmitter, NodeRef access, CheckMode mode);
    
};

class SpectreV1_Classic_Detector final: public SpectreV1_Detector {
public:
    SpectreV1_Classic_Detector(aeg::AEG& aeg, Solver& solver): SpectreV1_Detector(aeg, solver) {}
    
private:
    virtual DepVec deps() const override final;
    virtual std::string name() const override final { return "SpectreV1Classic"; }
};

class SpectreV1_Control_Detector final: public SpectreV1_Detector {
public:
    SpectreV1_Control_Detector(aeg::AEG& aeg, Solver& solver): SpectreV1_Detector(aeg, solver) {}
private:
    virtual DepVec deps() const override final;
    virtual std::string name() const override final { return "SpectreV1Control"; }
};

}
