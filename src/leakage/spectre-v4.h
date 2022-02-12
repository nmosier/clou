#pragma once

#include "leakage.h"

namespace lkg {

class SpectreV4_Detector: public DetectorJob {
public:
    SpectreV4_Detector(aeg::AEG& aeg, z3::context& local_ctx, z3::solver& solver, NodeRef candidate_transmitter, std::vector<std::pair<Leakage, std::string>>& leaks): DetectorJob(aeg, local_ctx, solver, candidate_transmitter, leaks) {}
    
    static DepVec get_deps();
    
private:
    std::vector<float> sats;
    std::vector<float> unknowns;
    std::vector<float> unsats;
    static constexpr float scale = 1.5f;
    
    virtual DepVec deps() const override { return get_deps(); }
    virtual void run_postdeps(const NodeRefVec& vec, CheckMode mode) override;
    virtual std::optional<float> get_timeout() const override;
    virtual void set_timeout(z3::check_result check_res, float secs) override;
    
    virtual void run_transmitter(NodeRef transmitter, CheckMode mode) override;
    void run_bypassed_store(NodeRef load, const NodeRefVec& vec, CheckMode mode);
    void run_bypassed_store_fast(NodeRef load, const NodeRefVec& vec, CheckMode mode);
    void run_sourced_store(NodeRef load, NodeRef bypassed_store, const NodeRefVec& vec, CheckMode mode);
    void check_solution(NodeRef load, NodeRef bypassed_store, NodeRef sourced_store, const NodeRefVec& vec, CheckMode mode);
    virtual std::string name() const override final { return "SpectreV4"; }
    virtual void entry(NodeRef, CheckMode) override;
    
    bool different_names(NodeRef load, NodeRef source) const;
    
    bool filter() const;
    
    void constrain_lsq(NodeRef bypassed_store, NodeRef load);
};

}
