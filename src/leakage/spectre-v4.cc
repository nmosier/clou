#include "spectre-v4.h"
#include "cfg/expanded.h"

namespace lkg {


void SpectreV4_Detector::run_() {
    for_each_transmitter(aeg::Edge::ADDR, [&] (NodeRef transmitter, CheckMode mode) {
        leak.transmitter = transmitter;
        if (use_lookahead && !lookahead([&] () {
            run_load(transmitter, CheckMode::FAST);
        })) {
            return;
        }
        run_load(transmitter, mode);
    });
}

void SpectreV4_Detector::run_load(NodeRef access, CheckMode mode) {
    // bind loads
    {
        const auto addrs = aeg.get_nodes(Direction::IN, access, aeg::Edge::ADDR);
        if (addrs.empty() && mode == CheckMode::SLOW) {
            std::cerr << "no addrs\n";
        }
        for (const auto& addr : addrs) {
            z3_cond_scope;
            const NodeRef load = addr.first;
            leak.load = load;
            const auto edge = push_edge({
                .src = load,
                .dst = access,
                .kind = aeg::Edge::ADDR,
            });
            const std::string desc = util::to_string(load, " -addr-> ", access);
            if (mode == CheckMode::SLOW) {
                solver.add(addr.second, desc.c_str());
                solver.add(aeg.lookup(load).trans, "load.trans");
            }
            const auto action = util::push(actions, desc);
            if (mode == CheckMode::FAST || solver.check() != z3::unsat) {
                run_bypassed_store(mode);
            } else {
                if (mode == CheckMode::SLOW) {
                    logv(1, "unsat, backtracking\n");
                }
            }
        }
    }
    
    // traceback
    traceback(access, [&] (NodeRef load, CheckMode mode) {
        if (mode == CheckMode::SLOW) {
            logv(1, "traceback " << load << "\n");
        }
        run_load(load, mode);
    }, mode);
}


void SpectreV4_Detector::run_bypassed_store(CheckMode mode) {
    if (mode == CheckMode::SLOW) {
        std::cerr << __FUNCTION__ << "\n";
    }
    traceback_rf(leak.load, [&] (const NodeRef bypassed_store, CheckMode mode) {
        // store can't be bypased if older than stb_size
        if (bypassed_store != aeg.entry && !aeg.may_source_stb(leak.load, bypassed_store)) { return; }
        
        z3_cond_scope;
        leak.bypassed_store = bypassed_store;
        const auto edge = push_edge({
            .src = bypassed_store,
            .dst = leak.load,
            .kind = aeg::Edge::ADDR,
        });
        run_sourced_store(mode);
    }, mode);
}


void SpectreV4_Detector::run_sourced_store(CheckMode mode) {
    if (mode == CheckMode::SLOW) {
        std::cerr << __FUNCTION__ << "\n";
    }
    
    // Only process candidate source stores that can possibly appear before the bypassed store in program order
    for (const NodeRef sourced_store : aeg.po.reverse_postorder()) {
        if (sourced_store == leak.bypassed_store) { break; } // stores must be distinct
        
        // NOTE: Is this a sound assumption to make?
        if (sourced_store != aeg.entry && !aeg.may_source_stb(leak.load, sourced_store)) { continue; } // would be outside of store buffer
        
        // Another approximation of is_ancestor()
        if (aeg.lookup(sourced_store).stores_in > aeg.lookup(leak.bypassed_store).stores_in) {
            continue;
        }
        
        z3_cond_scope;

        const aeg::Node& sourced_store_node = aeg.lookup(sourced_store);
        if (!sourced_store_node.may_write()) { continue; }
        if (mode == CheckMode::SLOW) {
            solver.add(sourced_store_node.write, "sourced_store.write");
        }
        
        leak.sourced_store = sourced_store;
        
        if (mode == CheckMode::SLOW) {
            const z3::expr same_addr = aeg::Node::same_addr(sourced_store_node, aeg.lookup(leak.load));
            solver.add(same_addr, "load.addr == sourced_store.addr");
            solver.add(aeg.rfx_exists(sourced_store, leak.load), "load -rfx-> sourced_store");
        }
        
        const auto action = util::push(actions, util::to_string("sourced ", sourced_store));
        
        if (mode == CheckMode::SLOW) {
            const z3::check_result res = solver.check();
            switch (res) {
                case z3::sat: {
                    const auto edge = push_edge(EdgeRef {
                        .src = leak.load,
                        .dst = sourced_store,
                        .kind = aeg::Edge::RFX,
                    });
                    output_execution(leak.leakage());
                    break;
                }
                case z3::unsat: {
                    logv(0, __FUNCTION__ << ": backtrack: unsat\n");
                    break;
                }
                case z3::unknown: {
                    std::cerr << "Z3 ERROR: unknown: " << solver.reason_unknown() << "\n";
                    std::abort();
                }
                default: std::abort();
            }
        } else {
            throw lookahead_found();
        }
    }
}



}
