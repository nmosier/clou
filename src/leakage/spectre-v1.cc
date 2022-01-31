#include "spectre-v1.h"
#include "util/timer.h"
#include "aeg/node.h"
#include "aeg/aeg.h"

namespace lkg {

std::optional<float> SpectreV1_Detector::get_timeout() const {
    // return std::nullopt;
    if (unsats.empty()) {
        return 1 * 60.f;
    } else {
        return util::average(unsats) * 5;
    }
}

void SpectreV1_Detector::set_timeout(z3::check_result check_res, float secs) {
    switch (check_res) {
        case z3::sat:
            sats.push_back(secs);
            break;
            
        case z3::unsat:
            unsats.push_back(secs);
            break;
            
        case z3::unknown:
            unknowns.push_back(secs);
            break;
            
        default: std::abort();
    }
}

void SpectreV1_Detector::entry(NodeRef candidate_transmitter, CheckMode mode) {
    run2(candidate_transmitter, candidate_transmitter, mode);
}


DetectorJob::DepVec SpectreV1_Classic_Detector::get_deps() {
    return {{aeg::Edge::ADDR_GEP, aeg::ExecMode::EXEC}, {aeg::Edge::ADDR, aeg::ExecMode::TRANS}};
}

DetectorJob::DepVec SpectreV1_Control_Detector::get_deps() {
    return {{aeg::Edge::ADDR_GEP, aeg::ExecMode::EXEC}, {aeg::Edge::CTRL, aeg::ExecMode::TRANS}};
}


void SpectreV1_Detector::run2(NodeRef transmitter, NodeRef access, CheckMode mode) {
    traceback_deps(transmitter, [&] (NodeRefVec vec, CheckMode mode) {
        run_postdeps(vec, mode);
    }, mode);
}


void SpectreV1_Detector::run_transmitter(NodeRef transmitter, CheckMode mode) {
    run2(transmitter, transmitter, mode);
}

void SpectreV1_Detector::run_postdeps(const NodeRefVec& vec_, CheckMode mode) {
    NodeRefVec vec = vec_;
    
    /* check for leakage */
    if (mode == CheckMode::SLOW) {
        switch (solver_check(false)) {
            case z3::sat:
                break;
                
            case z3::unsat:
                return;
                
            case z3::unknown:
#if 0
                logv(1, __FUNCTION__ << ": Z3 error: unknown result: " << solver.reason_unknown() << "\n");
#else
                logv(1, __FUNCTION__ << ": Z3 error: unknown result\n");
#endif
                std::abort();
                
            default: std::abort();
        }
    }
    
    if (mode == CheckMode::FAST) {
        throw lookahead_found();
    }
    
    
    const z3::eval eval = solver_eval();
    
    using output::operator<<;
    logv(1, "spectre-v1 leak found: " << vec << "\n");
    
    const NodeRef universal_transmitter = vec.front();
    
    std::reverse(vec.begin(), vec.end());
    
    // ensure attacker taints
#if 0
    z3_scope;
    for (NodeRef ref : vec) {
        solver.add(translate(aeg.lookup(ref).attacker_taint.value));
    }
#else
    solver.add(translate(aeg.lookup(vec.front()).attacker_taint.value), "universal-leakage-attacker-taint");
    std::cerr << "checking universal leakage: " << solver.check() << "\n";
#endif
    
    output_execution(Leakage {
        .vec = vec,
        .transmitter = universal_transmitter
    });
}

}
