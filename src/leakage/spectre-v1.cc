#include "spectre-v1.h"
#include "timer.h"

namespace lkg {

void SpectreV1_Detector::run_() {
    for_each_transmitter(deps().back(), [&] (NodeRef transmitter, CheckMode mode) {
        run2(transmitter, transmitter, mode);
    });
}


SpectreV1_Classic_Detector::DepVec SpectreV1_Classic_Detector::deps() const {
    return {aeg::Edge::ADDR_GEP, aeg::Edge::ADDR};
}

SpectreV1_Control_Detector::DepVec SpectreV1_Control_Detector::deps() const {
    return {aeg::Edge::ADDR_GEP, aeg::Edge::CTRL};
}


void SpectreV1_Detector::run2(NodeRef transmitter, NodeRef access, CheckMode mode) {
    traceback_deps(transmitter, [&] (NodeRefVec vec, CheckMode mode) {
        if (mode == CheckMode::FAST) {
            throw lookahead_found();
        }
        
        z3_eval;
        
        using output::operator<<;
        logv(1, "spectre-v1 leak found: " << vec << "\n");
        
        const NodeRef universal_transmitter = vec.front();
        
        std::reverse(vec.begin(), vec.end());
        
        output_execution(Leakage {
            .vec = vec,
            .transmitter = universal_transmitter
        });
        
    }, mode);
}


void SpectreV1_Detector::run_transmitter(NodeRef transmitter, CheckMode mode) {
    run2(transmitter, transmitter, mode);
}

void SpectreV1_Detector::run_postdeps(const NodeRefVec& vec_, CheckMode mode) {
    NodeRefVec vec = vec_;
    /* check for leakage */
    
    if (mode == CheckMode::SLOW) {
        if (solver.check() == z3::unknown) {
            logv(1, __FUNCTION__ << ": Z3 error: unknown result: " << solver.reason_unknown() << "\n");
            std::abort();
        }
        assert(solver.check() == z3::sat);
    }
    
    if (mode == CheckMode::FAST) {
        throw lookahead_found();
    }
    
    z3_eval;
    
    using output::operator<<;
    logv(1, "spectre-v1 leak found: " << vec << "\n");
    
    const NodeRef universal_transmitter = vec.front();
    
    std::reverse(vec.begin(), vec.end());
    
    output_execution(Leakage {
        .vec = vec,
        .transmitter = universal_transmitter
    });
}

}
