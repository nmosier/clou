#pragma once

#include "leakage.h"

namespace lkg {

struct SpectreV1_Leakage {
    NodeRef load0, load1, transmitter2;
    
    Leakage leakage() const {
        return {
            .vec = {load0, load1, transmitter2},
            .transmitter = transmitter2,
        };
    }
};

class SpectreV1_Detector: public Detector {
public:
    
protected:
    virtual aeg::Edge::Kind cur_dep() const { return *(deps().rbegin() + loads.size()); }
    
    SpectreV1_Detector(aeg::AEG& aeg, Solver& solver): Detector(aeg, solver) {}

private:
    NodeRefVec loads;
    virtual void run_() override final;

    void run1(NodeRef transmitter, NodeRef access, CheckMode mode);
    void run2(NodeRef transmitter, NodeRef access, CheckMode mode);
    
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
