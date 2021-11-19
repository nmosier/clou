#include "spectre-v1.h"
#include "timer.h"

namespace lkg {

void SpectreV1_Detector::run1(NodeRef transmitter, NodeRef access, CheckMode mode) {
    if (mode == CheckMode::SLOW) {
        using output::operator<<;
        logv(1, __FUNCTION__ << ": transmitter=" << transmitter << " access=" << access << " loads=" << loads << "\n");
        
        // lookahead
        if (!lookahead([&] () {
            run1(transmitter, access, CheckMode::FAST);
        }) && use_lookahead) {
            return;
        }
    }

    // check if done
    if (loads.size() == deps().size()) {
        if (mode == CheckMode::SLOW) {
            z3::check_result res = solver.check();
            switch (res) {
                case z3::unsat: {
                    logv(1, "backtrack: unsat\n");
                    dbg::append_core(solver);
                    return;
                }
                case z3::sat: {
                    break;
                }
                case z3::unknown: {
                    std::cerr << "Z3 ERROR: result unknown: " << solver.reason_unknown() << "\n";
                    std::abort();
                }
                default: std::abort();
            }
            
            z3_eval;
            
            const SpectreV1_Leakage leak = {
                .load0 = loads.at(1),
                .load1 = loads.at(0),
                .transmitter2 = transmitter,
            };
            
            logv(1, "spectre-v1 leak found\n");
            
            output_execution(leak.leakage());
            return;
            
        } else {
            throw lookahead_found();
        }
    }
    
    assert(loads.size() < deps().size());
    
    /* try committing load */
    {
        const aeg::Edge::Kind dep_kind = cur_dep();
        const std::string dep_str = util::to_string(dep_kind);
        const auto deps = aeg.get_nodes(Direction::IN, access, dep_kind);
        
        if (deps.empty()) {
            goto label;
        }
        
        if (mode == CheckMode::SLOW) {
            if (solver.check() == z3::unsat) {
                logv(1, "backtrack: unsat\n");
                dbg::append_core(solver);
                return;
            }

            logv(1, "trying to commit " << access << " (" << deps.size() << " deps)\n");
        }
        
        for (const auto& dep : deps) {
            const NodeRef load = dep.first;
            const auto push_load = util::push(loads, load);
            if (!check_edge(load, access)) { continue; }
            z3_cond_scope;
            if (mode == CheckMode::SLOW) {
                assert_edge(load, access, dep.second, dep_kind);
            }
            const auto addr_edge = util::push(flag_edges, EdgeRef {
                .src = load,
                .dst = access,
                .kind = dep_kind,
            });
            
            const auto action = util::push(actions, util::to_string(load, " -", dep_kind, "-> ", access));
            if (mode == CheckMode::SLOW) {
                logv(1, __FUNCTION__ << ": committed " << load << " -" << dep_kind << "-> " << access << "\n");
            }
            run1(transmitter, load, mode);
        }
    }
    
    label:
    
    /* traceback */
    if (access != transmitter) {
        traceback(access, [&] (const NodeRef load, CheckMode mode) {
            std::optional<Timer> timer;
            if (mode == CheckMode::SLOW) {
                timer = Timer();
                logv(1, name() << ": traceback " << access << " to " << load << "\n");
            }
            run1(transmitter, load, mode);
        }, mode);
    }
}

void SpectreV1_Detector::run_() {
    for_each_transmitter(deps().back(), [&] (NodeRef transmitter, CheckMode mode) {
        run1(transmitter, transmitter, mode);
    });
}


SpectreV1_Classic_Detector::DepVec SpectreV1_Classic_Detector::deps() const {
    return {aeg::Edge::ADDR_GEP, aeg::Edge::ADDR};
}

SpectreV1_Control_Detector::DepVec SpectreV1_Control_Detector::deps() const {
    return {aeg::Edge::ADDR_GEP, aeg::Edge::CTRL};
}


}
