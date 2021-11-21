#pragma once

#include "leakage.h"

namespace lkg {

class SpectreV4_Detector: public Detector {
public:
    SpectreV4_Detector(aeg::AEG& aeg, Solver& solver): Detector(aeg, solver) {}
private:
    std::vector<float> sats;
    std::vector<float> unknowns;
    std::vector<float> unsats;
    static constexpr float scale = 1.5f;
    
    virtual DepVec deps() const override final;
    virtual void run_postdeps(const NodeRefVec& vec, CheckMode mode) override;
    virtual std::optional<float> get_timeout() const override;
    virtual void set_timeout(z3::check_result check_res, float secs) override;
    
    virtual void run_() override final;
    virtual void run_transmitter(NodeRef transmitter, CheckMode mode) override;
    void run_bypassed_store(NodeRef load, const NodeRefVec& vec, CheckMode mode);
    void run_bypassed_store_fast(NodeRef load, const NodeRefVec& vec, CheckMode mode);
    void run_sourced_store(NodeRef load, NodeRef bypassed_store, const NodeRefVec& vec, CheckMode mode);
    void check_solution(NodeRef load, NodeRef bypassed_store, NodeRef sourced_store, const NodeRefVec& vec, CheckMode mode);
    virtual std::string name() const override final { return "SpectreV4"; }
};

}
