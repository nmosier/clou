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

class SpectreV1_Detector: public DetectorJob {
public:
    
protected:
    SpectreV1_Detector(aeg::AEG& aeg, z3::context& local_ctx, z3::solver& solver, NodeRef candidate_transmitter, std::vector<std::pair<Leakage, std::string>>& leaks): DetectorJob(aeg, local_ctx, solver, candidate_transmitter, leaks) {}
        
private:
    NodeRefVec loads;
    virtual void entry(NodeRef ref, CheckMode mode) override final;
    
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
    SpectreV1_Classic_Detector(aeg::AEG& aeg, z3::context& local_ctx, z3::solver& solver, NodeRef candidate_transmitter, std::vector<std::pair<Leakage, std::string>>& leaks): SpectreV1_Detector(aeg, local_ctx, solver, candidate_transmitter, leaks) {}
    
    static DepVec get_deps();
    
private:
    virtual DepVec default_deps() const override { return get_deps(); }
    virtual std::string name() const override { return "SpectreV1Classic"; }
};

class SpectreV1_Control_Detector final: public SpectreV1_Detector {
public:
    SpectreV1_Control_Detector(aeg::AEG& aeg, z3::context& local_ctx, z3::solver& solver, NodeRef candidate_transmitter, std::vector<std::pair<Leakage, std::string>>& leaks): SpectreV1_Detector(aeg, local_ctx, solver, candidate_transmitter, leaks) {}
    
    static DepVec get_deps();
    
private:
    virtual DepVec default_deps() const override { return get_deps(); }
    virtual std::string name() const override final { return "SpectreV1Control"; }
};

}
