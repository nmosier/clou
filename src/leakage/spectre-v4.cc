#include "spectre-v4.h"
#include "cfg/expanded.h"

namespace lkg {


void SpectreV4_Detector::run_() {
    for_each_transmitter(aeg::Edge::ADDR, [&] (NodeRef transmitter, CheckMode mode) {
#if 0
        leak.transmitter = transmitter;
        if (use_lookahead && !lookahead([&] () {
            run_load(transmitter, CheckMode::FAST);
        })) {
            return;
        }
        run_load(transmitter, mode);
#else
        run_transmitter(transmitter, mode);
#endif
    });
}

void SpectreV4_Detector::run_transmitter(NodeRef transmitter, CheckMode mode) {
    const Deps deps = {aeg::Edge::ADDR, aeg::Edge::ADDR_GEP};
    traceback_deps(deps, transmitter, [&] (const NodeRefVec& vec, CheckMode mode) {
        
        const NodeRef load = vec.back();
        assert(aeg.lookup(load).may_read());
        
        run_bypassed_store(load, vec, mode);
        
    }, mode);
}

void SpectreV4_Detector::run_bypassed_store(NodeRef load, const NodeRefVec& vec, CheckMode mode) {
    traceback_rf(load, [&] (NodeRef bypassed_store, CheckMode mode) {
        // store can't be bypassed if older than stb_size
        if (bypassed_store != aeg.entry && !aeg.may_source_stb(leak.load, bypassed_store)) {
            return;
        }

        run_sourced_store(load, bypassed_store, vec, mode);
        
    }, mode);
}

void SpectreV4_Detector::run_sourced_store(NodeRef load, NodeRef bypassed_store, const NodeRefVec& vec, CheckMode mode) {
    const auto load_idx = aeg.po.postorder_idx(load);
    const auto bypassed_store_idx = aeg.po.postorder_idx(bypassed_store);
    assert(load_idx < bypassed_store_idx);
    
    for (NodeRef sourced_store : exec_window) {
        const auto sourced_store_idx = aeg.po.postorder_idx(sourced_store);
        const aeg::Node& sourced_store_node = aeg.lookup(sourced_store);
        
        // require sourced store to come before in postorder
        if (!(sourced_store_idx > bypassed_store_idx)) {
            continue;
        }
        
        // require that it may be store
        if (!sourced_store_node.may_write()) {
            continue;
        }
        
#if 0
        // check if would be outside of store buffer
        if (sourced_store != aeg.entry && !aeg.may_source_stb(load, sourced_store)) {
            continue;
        }
#endif
        
#if 0
        // Another approximation of is_ancestor()
        if (aeg.lookup(sourced_store).stores_in > aeg.lookup(leak.bypassed_store).stores_in) {
            continue;
        }
#endif
        
        z3_cond_scope;
        
        if (mode == CheckMode::SLOW) {
            const z3::expr same_addr = aeg::Node::same_addr(sourced_store_node, aeg.lookup(load));
            solver.add(same_addr, "load.addr == sourced_store.addr");
            solver.add(aeg.rfx_exists(sourced_store, load), "load -RFX-> sourced_store");
        }
        
        const auto action = util::push(actions, util::to_string("sourced ", sourced_store));
        
        if (mode == CheckMode::SLOW) {
            switch (solver.check()) {
                case z3::sat: {
                    const auto edge = push_edge(EdgeRef {
                        .src = leak.load,
                        .dst = sourced_store,
                        .kind = aeg::Edge::RFX,
                    });
                    const NodeRef universl_transmitter = vec.front();
                    const NodeRefVec vec2 = {sourced_store, bypassed_store, load, universl_transmitter};
                    output_execution(Leakage {
                        .vec = vec2,
                        .transmitter = universl_transmitter,
                    });
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
