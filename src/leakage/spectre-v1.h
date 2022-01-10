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
#if 0
    virtual aeg::Edge::Kind cur_dep() const { return *(deps().rbegin() + loads.size()); }
#endif
    
    SpectreV1_Detector(aeg::AEG& aeg, z3::solver& solver): Detector(aeg, solver) {}
        
private:
    NodeRefVec loads;
    virtual void run_() override final;
    
    virtual void run_transmitter(NodeRef transmitter, CheckMode mode) override;
    virtual void run_postdeps(const NodeRefVec& vec, CheckMode mode) override;

    void run2(NodeRef transmitter, NodeRef access, CheckMode mode);
    
    
    std::vector<float> sats;
    std::vector<float> unknowns;
    std::vector<float> unsats;
    
    virtual std::optional<float> get_timeout() const override;
    virtual void set_timeout(z3::check_result check_res, float secs) override;

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

}
