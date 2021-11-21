#include "spectre-v1.h"
#include "timer.h"

namespace lkg {

void SpectreV1_Detector::run_() {
    for_each_transmitter([&] (NodeRef transmitter, CheckMode mode) {
        run2(transmitter, transmitter, mode);
    });
}


Detector::DepVec SpectreV1_Classic_Detector::deps() const {
    return {{aeg::Edge::ADDR_GEP, aeg::ExecMode::EXEC}, {aeg::Edge::ADDR, aeg::ExecMode::TRANS}};
}

Detector::DepVec SpectreV1_Control_Detector::deps() const {
    return {{aeg::Edge::ADDR_GEP, aeg::ExecMode::EXEC}, {aeg::Edge::CTRL, aeg::ExecMode::TRANS}};
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
        assert(vec.size() >= 3);
        const NodeRef data_transmitter = vec.at(1);
        solver.add(aeg.lookup(data_transmitter).trans, "data_transmitter.trans");
    }
    
    if (mode == CheckMode::SLOW) {
        switch (solver.check()) {
            case z3::sat:
                break;
                
            case z3::unsat:
                return;
                
            case z3::unknown:
                logv(1, __FUNCTION__ << ": Z3 error: unknown result: " << solver.reason_unknown() << "\n");
                std::abort();
                
            default: std::abort();
        }
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
