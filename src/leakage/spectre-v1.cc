#include "spectre-v1.h"
#include "util/timer.h"
#include "aeg/node.h"
#include "aeg/aeg.h"
#include "cfg/expanded.h"

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
    
    
#if 1
    // find branch misspeculation
# if 0
    {
        z3_eval;
        
        NodeRef branch;
        for (const NodeRef ref : aeg.po.reverse_postorder()) {
            if (eval(translate(aeg.lookup(ref).arch))) {
                std::cerr << "arch: " << *aeg.lookup(ref).inst << "\n";
                branch = ref;
            }
            if (eval(translate(aeg.lookup(ref).trans))) {
                llvm::errs() << "trans: " << *aeg.lookup(ref).inst->get_inst() << "\n";
                
                for (const auto& edge : aeg.graph.fwd.at(branch).at(ref)) {
                    if (edge->kind == aeg::Edge::Kind::TFO) {
                        std::cerr << "tfo exists: " << eval(translate(edge->exists)) << "\n";
                    }
                }

                break;
            }
        }
        
        const aeg::Node& branch_node = aeg.lookup(branch);
        llvm::errs() << *branch_node.inst->get_inst() << "\n";
        const llvm::BranchInst *BI = llvm::cast<llvm::BranchInst>(branch_node.inst->get_inst());

        std::cerr << "branch predicate is tainted: " << eval(translate(branch_node.attacker_taint.value)) << "\n";

    }
# endif
    
    {
        z3::expr_vector taints {ctx()};
        for (const NodeRef ref : exec_window) {
            const auto& node = aeg.lookup(ref);
            if (const llvm::BranchInst *BI = llvm::dyn_cast_or_null<llvm::BranchInst>(node.inst->get_inst())) {
                auto tfos = aeg.get_nodes(Direction::OUT, ref, aeg::Edge::Kind::TFO);
                std::erase_if(tfos, [&] (const auto& p) {
                    return !exec_window.contains(p.first);
                });
                z3::expr_vector tfos_exist = z3::transform(ctx(), tfos.begin(), tfos.end(), [&] (const auto& p) {
                    return p.second && aeg.lookup(p.first).trans;
                });
                z3::expr pred = node.arch && node.attacker_taint.value && z3::mk_or(tfos_exist);
                taints.push_back(pred);
            }
        }
        
        solver_add(translate(z3::mk_or(taints)), "branch-tainted");
    }
    
    switch (solver_check(false)) {
        case z3::sat: break;
        case z3::unsat: return;
        case z3::unknown:
            std::cerr << "error: unknown result\n";
            std::abort();
    }
    
#endif
    
    output_execution(Leakage {
        .vec = vec,
        .transmitter = universal_transmitter
    });
}

}
