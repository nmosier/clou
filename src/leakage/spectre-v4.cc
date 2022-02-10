#include "spectre-v4.h"
#include "cfg/expanded.h"
#include "util/algorithm.h"
#include "aeg/node.h"
#include "aeg/aeg.h"

namespace lkg {

DetectorJob::DepVec SpectreV4_Detector::get_deps() {
    return DepVec {{aeg::Edge::ADDR, aeg::ExecMode::TRANS}, {aeg::Edge::ADDR, aeg::ExecMode::TRANS}};
}

void SpectreV4_Detector::entry(NodeRef candidate_transmitter, CheckMode mode) {
    run_transmitter(candidate_transmitter, mode);
}

void SpectreV4_Detector::run_transmitter(NodeRef transmitter, CheckMode mode) {
    traceback_deps(transmitter, [&] (const NodeRefVec& vec, CheckMode mode) {
        
        const NodeRef load = vec.back();
        assert(aeg.lookup(load).may_read());

        // make sure all traceback_deps are trans
        if (mode == CheckMode::SLOW) {
            z3::expr_vector vec_trans {ctx()};
            for (NodeRef ref : vec) {
                vec_trans.push_back(aeg.lookup(ref).trans);
            }
            solver_add(translate(z3::mk_and(vec_trans)), "traceback_deps.trans");
        }
        
        /* add <ENTRY> -RFX-> load */
        if (!spectre_v4_mode.concrete_sourced_stores) {
            if (mode == CheckMode::SLOW) {
                solver_add(translate(aeg.rfx_exists(aeg.entry, load)), "entry -RFX-> load");
            }
        }
        
        run_bypassed_store(load, vec, mode);
        
    }, mode);
}

bool SpectreV4_Detector::filter() const {
    for (const auto& action : actions) {
        if (action.edge == aeg::Edge::RF) {
            // if source doesn't have incoming ADDR dependency, then apply LLVM's AA
            const auto addrs = aeg.get_nodes(Direction::IN, action.src, aeg::Edge::Kind::ADDR);
            if (addrs.empty()) {
                if (aeg.check_alias(action.src, action.dst) == llvm::NoAlias) {
                    llvm::errs() << "SpectreV4_Detector::filter(): filtered\n";
                    return true;
                }
            }
        }
    }
    
    return false;
}

void SpectreV4_Detector::run_bypassed_store(NodeRef load, const NodeRefVec& vec, CheckMode mode) {
    
    
    /*
     * TODO: in fast mode, Don't even need to trace back RF… just need to find ONE store that can be sourced,
     * Basically, we can just OR all stores together.
     */
    // TODO: give this its own variable?
    if (!spectre_v4_mode.concrete_sourced_stores) {
        run_bypassed_store_fast(load, vec, mode);
        return;
    }
    
    /* in "concrete sourced stores" mode, do optional traceback */
#define ADDITONAL_TRACEBACK 1
    
    if (mode == CheckMode::SLOW) {
        // check if sat
        if (solver_check() == z3::unsat) {
            logv(1, __FUNCTION__ << ":" << __LINE__ << ": backtrack: unsat\n");
            return;
        }
    }
    
    traceback_rf(load, aeg::ExecMode::ARCH, [&] (NodeRef bypassed_store, CheckMode mode) {
        // store can't be bypassed if older than stb_size
        if (bypassed_store == aeg.entry) { return; }
        
        if (!aeg.may_source_stb(load, bypassed_store)) {
            return;
        }
        
        if (spectre_v4_mode.diff_names && !different_names(load, bypassed_store)) {
            llvm::errs() << "Skipping due to same name\n";
            return;
        }
        
        const auto& node = aeg.lookup(bypassed_store);
        
        if (mode == CheckMode::SLOW) {
            solver_add(translate(node.arch), "bypassed_store.arch");
            
            // check if sat
            if (solver_check() == z3::unsat) {
                logv(1, __FUNCTION__ << ":" << __LINE__ << ": backtrack: unsat\n");
                return;
            }
        }
        
        if (spectre_v4_mode.concrete_sourced_stores) {
            run_sourced_store(load, bypassed_store, vec, mode);
        } else {
            check_solution(load, bypassed_store, aeg.entry, vec, mode);
        }
        
    }, mode);
    
#if ADDITONAL_TRACEBACK
    {
        // TODO: not sure if this is right
        traceback(load, aeg::ExecMode::ARCH, [&] (NodeRef load, CheckMode mode) {
            NodeRefVec vec_ = vec;
            vec_.push_back(load);
            if (mode == CheckMode::SLOW) {
                solver_add(translate(aeg.lookup(load).trans), util::to_string(load, ".trans").c_str());
            }
            run_bypassed_store(load, vec_, mode);
        }, mode);
    }
#endif
    
}

bool SpectreV4_Detector::different_names(NodeRef load, NodeRef store) const {
    const aeg::Node& load_node = aeg.lookup(load);
    const aeg::Node& store_node = aeg.lookup(store);
    const llvm::Value *load_addr = load_node.get_memory_address_pair().first;
    const llvm::Value *store_addr = store_node.get_memory_address_pair().first;
    const auto& load_id = aeg.po.lookup(load).id;
    const auto& store_id = aeg.po.lookup(store).id;
    
#if 0
    if (llvm::isa<llvm::Argument>(load_addr) && llvm::isa<llvm::Argument>(store_addr)) {
        // same inlining context
        if (load_id && store_id && load_id->func == store_id->func) {
            return load_addr != store_addr;
        } else {
            return true;
        }
    } else if (llvm::isa<llvm::Constant>(load_addr) && llvm::isa<llvm::Constant>(store_addr)) {
        return load_addr == store_addr;
    }
#endif
    
    if (llvm::isa<llvm::AllocaInst>(load_addr) && llvm::isa<llvm::AllocaInst>(store_addr)) {
        if (load_id && store_id && load_id->func == store_id->func) {
            return load_addr != store_addr;
        } else {
            return true;
        }
    }
        
    return true;
}

void SpectreV4_Detector::run_bypassed_store_fast(NodeRef load, const NodeRefVec& vec, CheckMode mode) {
    NodeRefVec todo = {load};
    NodeRefSet seen;
    z3::expr_vector exprs(ctx());
    exprs.push_back(ctx().bool_val(false));
    
    while (!todo.empty()) {
        NodeRef bypassed_store = todo.back();
        todo.pop_back();
        if (!seen.insert(bypassed_store).second) { continue; }
        
        const auto& node = aeg.lookup(bypassed_store);
        
        // if it is a write, then check whether it can be bypassed
        if (node.may_write() && aeg.may_source_stb(load, bypassed_store)) {
            if (!spectre_v4_mode.diff_names || different_names(load, bypassed_store)) {
                if (mode == CheckMode::SLOW) {
                    exprs.push_back(node.arch && node.write && aeg.same_addr(bypassed_store, load));
                }
                if (mode == CheckMode::FAST) {
                    throw lookahead_found();
                }
            }
        }
                
        util::copy(aeg.po.po.rev.at(bypassed_store), std::back_inserter(todo));
    }

    if (mode == CheckMode::SLOW) {
        // TODO: make it so that bypassed store is optoinal
      
        solver_add(translate(z3::mk_or(exprs)), "bypassed_store");
        check_solution(load, aeg.entry, aeg.entry, vec, mode);
    }
}

void SpectreV4_Detector::check_solution(NodeRef load, NodeRef bypassed_store, NodeRef sourced_store, const NodeRefVec& vec, CheckMode mode) {
    // try to filter
    if (filter()) {
        return;
    }
    
    if (mode == CheckMode::SLOW) {
        switch (solver_check(false)) {
            case z3::sat: {
                const auto edge = push_edge(EdgeRef {
                    .src = sourced_store,
                    .dst = load,
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

void SpectreV4_Detector::run_sourced_store(NodeRef load, NodeRef bypassed_store, const NodeRefVec& vec, CheckMode mode) {
    [[maybe_unused]] const auto load_idx = aeg.po.postorder_idx(load);
    const auto bypassed_store_idx = aeg.po.postorder_idx(bypassed_store);
    assert(load_idx < bypassed_store_idx);
    
    NodeRefSet sourced_store_candidates = exec_window;
    
    for (NodeRef sourced_store : sourced_store_candidates) {
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
        
        z3_cond_scope;
        
        if (mode == CheckMode::SLOW) {
            const z3::expr same_addr = aeg.same_addr(sourced_store, load);
            solver_add(translate(same_addr), "load.addr == sourced_store.addr");
            solver_add(translate(aeg.rfx_exists(sourced_store, load)), "load -RFX-> sourced_store");
        }
        
#if 0
        const auto action = util::push(actions, util::to_string("sourced ", sourced_store));
#else
#if 0
        const auto action = util::push(actions, {.src = sourced_store, .edge = aeg::Edge::Kind::RFX, .dst = load});
        std::abort();
#endif
#endif
        
        check_solution(load, bypassed_store, sourced_store, vec, mode);
        
    }
}

void SpectreV4_Detector::run_postdeps(const NodeRefVec& vec, CheckMode mode) {
    const NodeRef load = vec.back();
    assert(aeg.lookup(load).may_read());
    run_bypassed_store(load, vec, mode);
}


std::optional<float> SpectreV4_Detector::get_timeout() const {
    // return std::nullopt;
    if (unsats.empty()) {
        return 5.f;
    } else {
        return util::average(unsats) * 5;
    }
}

void SpectreV4_Detector::set_timeout(z3::check_result check_res, float secs) {
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


}
