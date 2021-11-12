#include "spectre-v1.h"
#include "timer.h"

namespace lkg {

void SpectreV1_Detector::run1(NodeRef transmitter, NodeRef access, CheckMode mode) {
    if (mode == CheckMode::SLOW) {
        std::cerr << __FUNCTION__ << ": transmitter=" << transmitter << " access=" << access << " loads=" << loads << "\n";
        
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
            if (solver.check() != z3::sat) {
                trace("backtrack: unsat");
                return;
            }
            
            z3_eval;
            
            const SpectreV1_Leakage leak = {
                .load0 = loads.at(1),
                .load1 = loads.at(0),
                .transmitter2 = transmitter,
            };
            
            std::cerr << "spectre-v1 leak found\n";
            
            output_execution(leak);
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
            if (solver.check() != z3::sat) {
                trace("backtrack: unsat");
                std::cerr << "mark:unsat\n";
                dbg::append_core(solver);
                return;
            }

            trace("trying to commit %lu (%zu deps)", access, deps.size());
        }
        
        for (const auto& dep : deps) {
            z3_cond_scope;
            const NodeRef load = dep.first;
            const auto push_load = util::push(loads, load);
            if (mode == CheckMode::SLOW) {
                solver.add(dep.second, util::to_string(load, " -", dep_kind, "-> ", access).c_str());
            }
            const auto addr_edge = util::push(flag_edges, EdgeRef {
                .src = load,
                .dst = access,
                .kind = aeg::Edge::ADDR,
            });
            
            const auto action = util::push(actions, util::to_string(load, " -", dep_kind, "-> ", access));
            if (mode == CheckMode::SLOW) {
                std::cerr << __FUNCTION__ << ": committed " << load << " -" << dep_kind << "-> " << access << "\n";
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
                std::cerr << name() << ": traceback " << access << " to " << load << "\n";
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
