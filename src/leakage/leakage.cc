#include <fstream>

#include <gperftools/profiler.h>

#include "aeg/aeg.h"
#include "timer.h"
#include "fork_work_queue.h"
#include "hash.h"
#include "util/llvm.h"
#include "fol.h"
#include "cfg/expanded.h"
#include "util/output.h"
#include "util/iterator.h"
#include "leakage.h"
#include "mon/client.h"
#include "mon/proto.h"

#define z3_cond_scope std::optional<z3::scope> scope = (mode == CheckMode::SLOW) ? std::make_optional(solver) : std::optional<z3::solver>()

#define do_lookahead(call) \
if (!lookahead([&] () { call; }) && use_lookahead) { \
return; \
}


/* For each speculative-dst addr edge, find all leakage coming out of it.
 * Any rfx edges, it's leakage, as long as the tail of the rfx edge is a READ.
 * Any frx edges, it's leakage, as ....
 * hard to check for frx edges
 *
 
 Is there a way to avoid iterating over all co, fr edges?
 Can we somehow check if the total order differs?
 In the case of co, a co edge won't have a corresponding cox edge in one scenario: the write doesn't act as an xswrite (silent store). This can be checked for by iterating over writes and ensuring they have corresponding xswrites.
 ... a corresponding rfx edge if there is an intervening xswrite.
 ... a correspondinf frx edge if it happens before the first write?
 
 co, ~rfx case: this is due to a deviation in co and cox order OR reads perturbing cache state (really the same -- order of writes vs. xswrites disagree).
 One possible way to make this efficient would be to enforce that the order of memory accesses and xstate accesses are on the same timescale. Then, whenever they differ...
 Could instantiate two total order relations and assert that they are equal on the interval [0, size()) x [0, size()). But of course there would always be a counterexample. Would need to restrict it to tails of address relations, or more generally, a subset of unsafe instructions. Could say forall dst, if dst in addr_dsts, then forall idx in [0, size()), (dst, idx) in co iff (dst, idx) in cox.
 
 Are forall statements over bitvectors simpler?
 I should at least try this out.
 
 For now, just do naive implementation.
 The new flow should do the following:
 - leakage() function outputs an example of leakage.
 - static analysis of concrete graph discovers all leakage in graph. Also detects cause, so outputs new constraints to forbid this kind of leakage.
 */

/*
 Doing these leakage checks is tautological.
 
 rf/rfx: There will only ever be leakage involving rfx and a transient address dependency if the latter is an xswrite.
 co/cox: There will only ever be leakage ... if the latter is an xswrite.
 fr/frx: There will only ever be leakage ... if the latter is an xswrite.
 
 If we restrict our view to address dependencies,
 
 RF
 Either (A) there is a subsequent instruction in xsaccess order that rf's the same addr/xstate, or (B) the program bottom rf's the same addr/xstate. Either way, there is rf/rfx leakage.
 
 CO
 Either (A) there is a subsequent instruction in xsaccess order that co's the same addr/xstate, or (B) there is not. co/cox leakage is thus optional.
 
 FR
 Either (A) there is a subsequent instruction in xsaccess order that fr's the same addr/xstate, or (B) there is not. fr/frx leakage is thus optional.
 
 
 
 */

namespace aeg {

unsigned AEG::leakage(z3::solver& solver) {
    switch (leakage_class) {
        case LeakageClass::SPECTRE_V4: {
            auto detector = std::make_unique<lkg::SpectreV4_Detector>(*this, solver);
            detector->run();
            return 0;
        }
          
        case LeakageClass::SPECTRE_V1: {
            std::unique_ptr<lkg::Detector> detector;
            switch (spectre_v1_mode.mode) {
                case SpectreV1Mode::Mode::CLASSIC:
                    detector = std::make_unique<lkg::SpectreV1_Classic_Detector>(*this, solver);
                    break;
                case SpectreV1Mode::Mode::BRANCH_PREDICATE:
                    detector = std::make_unique<lkg::SpectreV1_Control_Detector>(*this, solver);
                    break;
                default: std::abort();
            }
            detector->run();
            return 0;
        }
            
        default: std::abort();
    }
}

}


namespace lkg {

/* LEAKAGE DETECTOR METHODS */

Detector::Detector(aeg::AEG& aeg, z3::solver& solver): aeg(aeg), solver(solver), init_mem(z3::const_array(ctx().int_sort(), ctx().int_val(static_cast<unsigned>(aeg.entry)))), mems(get_mems()), partial_order(aeg.po), rf_solver(z3::duplicate(solver)) {
    aeg.po.reverse_postorder(std::back_inserter(order));
}

z3::expr Detector::mem(NodeRef ref) const {
    const auto it = mems.find(ref);
    if (it == mems.end()) {
        return init_mem;
    } else {
        return it->second;
    }
}

Detector::Mems Detector::get_mems() {
    z3::context& ctx = this->ctx();
    auto& po = aeg.po;
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));

    Mems ins;
    Mems outs = {{aeg.entry, init_mem}};
    for (const NodeRef cur : order) {
        if (cur == aeg.entry) { continue; }
        const auto& cur_node = aeg.lookup(cur);
        
        auto tfos = aeg.get_nodes(Direction::IN, cur, aeg::Edge::TFO);
        z3::expr mem {ctx};
        if (tfos.empty()) {
            mem = init_mem;
        } else {
            auto tfo_it = tfos.begin();
            mem = outs.at(tfo_it->first);
            ++tfo_it;
            mem = std::accumulate(tfo_it, tfos.end(), mem, [&] (const z3::expr& acc, const auto& tfo) -> z3::expr {
                return z3::ite(tfo.second, outs.at(tfo.first), acc);
            });
        }
        
        ins.emplace(cur, mem);
        
        if (cur_node.may_write()) {
            mem = z3::conditional_store(mem, cur_node.get_memory_address(), ctx.int_val(static_cast<unsigned>(cur)), cur_node.exec() && cur_node.write);
        }
        
        outs.emplace(cur, mem);
    }
    
    return ins;
}

Detector::Mems Detector::get_mems(const NodeRefSet& set) {
    NodeRefVec order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    
    Mems ins;
    Mems outs;
    const auto outs_at = [&] (const NodeRef ref) -> z3::expr {
        const auto it = outs.find(ref);
        if (it == outs.end()) {
            return init_mem;
        } else {
            return it->second;
        }
    };
    for (const NodeRef ref : order) {
        if (ref == aeg.entry ||
            set.find(ref) == set.end()) {
            continue;
        }
        const aeg::Node& node = aeg.lookup(ref);
        
        const auto tfos = aeg.get_nodes(Direction::IN, ref, aeg::Edge::TFO);
        z3::expr mem {ctx()};
        if (tfos.empty()) {
            mem = init_mem;
        } else {
            auto tfo_it = tfos.begin();
            mem = outs_at(tfo_it->first);
            ++tfo_it;
            mem = std::accumulate(tfo_it, tfos.end(), mem, [&] (const z3::expr& acc, const auto& tfo) -> z3::expr {
                return z3::ite(tfo.second, outs_at(tfo.first), acc);
            });
        }
        
        ins.emplace(ref, mem);
        
        if (node.may_write()) {
            mem = z3::conditional_store(mem, node.get_memory_address(), ctx().int_val(static_cast<unsigned>(ref)), node.exec() && node.write);
        }
        
        outs.emplace(ref, mem);
    }
    
    return ins;
}

bool Detector::lookahead(std::function<void ()> thunk) {
    try {
        thunk();
        lookahead_tmp = false;
        return false;
    } catch (const lookahead_found&) {
        lookahead_tmp = true;
        return true;
    }
}

void Detector::traceback_rf(NodeRef load, std::function<void (NodeRef, CheckMode)> func, CheckMode mode) {
    // Sources new_sources;
    for (const auto& store_pair : rf_sources(load)) {
        const NodeRef store = store_pair.first;
        
        z3_cond_scope;
        const std::string desc = util::to_string(store, " -rf-> ", load);
        if (mode == CheckMode::SLOW) {
            const z3::expr& cond = store_pair.second;
            solver.add(cond, desc.c_str());
#if 0
            // NOTE: not checking actually speeds up execution by 0.5X
            if (solver.check() != z3::sat) { continue; }
#endif
        }
        
        const auto action = util::push(actions, desc);

        // new_sources.insert(store_pair);
        // TODO: need to separately check if this is due to something else
        func(store, mode);
    }
}

void Detector::traceback_edge(aeg::Edge::Kind kind, NodeRef ref, std::function<void (NodeRef, CheckMode)> func, CheckMode mode) {
    const auto edges = aeg.get_nodes(Direction::IN, ref, kind);
    for (const auto& edge : edges) {
        z3_cond_scope;
        if (mode == CheckMode::SLOW) {
            solver.add(edge.second, util::to_string(edge.first, " -", kind, "-> ", ref).c_str());
        }
        const auto action = util::push(actions, util::to_string(edge.first, " -", kind, "-> ", ref));
        func(edge.first, mode);
    }
}

void Detector::traceback(NodeRef load, std::function<void (NodeRef, CheckMode)> func, CheckMode mode) {
    const aeg::Node& load_node = aeg.lookup(load);

    z3_cond_scope;
    if (mode == CheckMode::SLOW) {
        solver.add(load_node.exec() && load_node.read, util::to_string(load, ".read").c_str());
        if (solver.check() != z3::sat) { return; }
    }
    
    if (traceback_depth == max_traceback) {
        std::cerr << "backtracking: max traceback depth (" << max_traceback << ")\n";
        return;
    }
    
    const auto inc_depth = util::inc_scope(traceback_depth);
    
    // traceback via rf.data
    traceback_rf(load, [&] (const NodeRef store, CheckMode mode) {
        traceback_edge(aeg::Edge::DATA, store, func, mode);
    }, mode);
    
    // traceback via addr
    traceback_edge(aeg::Edge::ADDR, load, func, mode);
}

template <typename Derived>
void Leakage<Derived>::print_short(std::ostream& os) const {
    const auto v = static_cast<const Derived&>(*this).vec();
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (it != v.begin()) {
            os << " ";
        }
        os << *it;
    }
}

template <typename Derived>
void Leakage<Derived>::print_long(std::ostream& os, const aeg::AEG& aeg) const {
    const auto v = static_cast<const Derived&>(*this).vec();
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (it != v.begin()) {
            os << "; ";
        }
        os << *aeg.lookup(*it).inst;
    }
}


void Detector::for_each_transmitter(aeg::Edge::Kind kind, std::function<void (NodeRef)> func) {
    NodeRefSet candidate_transmitters;
    aeg.for_each_edge(kind, [&] (NodeRef, NodeRef ref, const aeg::Edge&) {
        candidate_transmitters.insert(ref);
    });
    
    std::size_t i = 0;
    for (NodeRef transmitter : candidate_transmitters) {
        Timer timer;
        
        std::cerr << ++i << "/" << candidate_transmitters.size() << "\n";
        
        if (client) {
            mon::Message msg;
            auto *progress = msg.mutable_func_progress();
            progress->mutable_func()->set_name(aeg.po.function_name());
            const float frac = static_cast<float>(i) / static_cast<float>(candidate_transmitters.size());
            progress->set_frac(frac);
            client.send(msg);
        }
        
        if (transmitters.find(aeg.lookup(transmitter).inst->get_inst()) != transmitters.end()) {
            continue;
        }
        
        const auto action = util::push(actions, util::to_string("transmitter ", transmitter));
        
        z3_scope;
        const aeg::Node& transmitter_node = aeg.lookup(transmitter);
        
        if (aeg.exits.find(transmitter) != aeg.exits.end()) {
            continue;
        }
        
#define VERIFY_SAT 0
        
#if VERIFY_SAT
        assert(solver.check() == z3::sat);
#endif
        
        // require transmitter is access
        solver.add(transmitter_node.access(), "transmitter.access");
        
#if VERIFY_SAT
        assert(solver.check() == z3::sat);
#endif
        
        // require transmitter.trans
        solver.add(transmitter_node.trans, "transmitter.trans");
        
        // window size
        const auto saved_mems = util::save(mems);
        {
            z3::expr_vector src {ctx()};
            z3::expr_vector dst {ctx()};
            NodeRefSet window;
            aeg.for_each_pred_in_window(transmitter, window_size, [&window] (NodeRef ref) {
                window.insert(ref);
            }, [&] (NodeRef ref) {
                const aeg::Node& node = aeg.lookup(ref);
                solver.add(!node.exec(), util::to_string("window-exclude-", ref).c_str());
                src.push_back(node.arch); dst.push_back(ctx().bool_val(false));
                src.push_back(node.trans); dst.push_back(ctx().bool_val(false));
            });
#if 0
            for (auto& p : mems) {
                z3::expr& mem = p.second;
                mem = mem.substitute(src, dst).simplify();
            }
#endif
#if 1
            mems = get_mems(window);
#endif
        }
        
        if (solver.check() == z3::sat) {
            try {
                func(transmitter);
            } catch (const next_transmitter& e) {
                // continue
            }
        } else {
            std::cerr << "skipping transmitter\n";
            std::cerr << "access: " << transmitter_node.access() << "\n";
            std::cerr << "trans: " << transmitter_node.trans << "\n";
        }
    }
}

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
    for_each_transmitter(deps().back(), [&] (const NodeRef transmitter) {
        run1(transmitter, transmitter, CheckMode::SLOW);
    });
}


SpectreV1_Classic_Detector::DepVec SpectreV1_Classic_Detector::deps() const {
    return {aeg::Edge::ADDR_GEP, aeg::Edge::ADDR};
}

SpectreV1_Control_Detector::DepVec SpectreV1_Control_Detector::deps() const {
    return {aeg::Edge::ADDR_GEP, aeg::Edge::CTRL};
}


void SpectreV4_Detector::run_() {
    for_each_transmitter(aeg::Edge::ADDR, [&] (const NodeRef transmitter) {
        std::cerr << "transmitter " << transmitter << " (" << solver.assertions().size() << " assertions)\n";
        leak.transmitter = transmitter;
        run_load(transmitter);
    });
}

void SpectreV4_Detector::run_load(NodeRef access) {
    // bind loads
    {
        const auto addrs = aeg.get_nodes(Direction::IN, access, aeg::Edge::ADDR);
        if (addrs.empty()) {
            std::cerr << "no addrs\n";
        }
        for (const auto& addr : addrs) {
            z3_scope;
            const NodeRef load = addr.first;
            leak.load = load;
            const auto edge = push_edge({
                .src = load,
                .dst = access,
                .kind = aeg::Edge::ADDR,
            });
            const std::string desc = util::to_string(load, " -addr-> ", access);
            solver.add(addr.second, desc.c_str());
            solver.add(aeg.lookup(load).trans, "load.trans");
            const auto action = util::push(actions, desc);
            if (solver.check() == z3::sat) {
                run_bypassed_store();
            } else {
                std::cerr << "unsat, backtracking\n";
            }
        }
    }
    
    // traceback
    traceback(access, [&] (NodeRef load, CheckMode mode) {
        if (mode == CheckMode::FAST) { throw lookahead_found(); }
        std::cerr << "traceback " << load << "\n";
        run_load(load);
    }, CheckMode::SLOW);
}


void SpectreV4_Detector::run_bypassed_store() {
    std::cerr << __FUNCTION__ << "\n";
    traceback_rf(leak.load, [&] (const NodeRef bypassed_store, CheckMode mode) {
        if (mode == CheckMode::FAST) { throw lookahead_found(); }
        
        // store can't be bypased if older than stb_size
        if (bypassed_store != aeg.entry && !aeg.may_source_stb(leak.load, bypassed_store)) { return; }
        
        z3_scope;
        leak.bypassed_store = bypassed_store;
        const auto edge = push_edge({
            .src = bypassed_store,
            .dst = leak.load,
            .kind = aeg::Edge::ADDR,
        });
        run_sourced_store();
    }, CheckMode::SLOW);
}


void SpectreV4_Detector::run_sourced_store() {
    std::cerr << __FUNCTION__ << "\n";
    
    // Only process candidate source stores that can possibly appear before the bypassed store in program order
    for (const NodeRef sourced_store : order) {
        if (sourced_store == leak.bypassed_store) { break; } // stores must be distinct
        
        // NOTE: Is this a sound assumption to make?
        if (sourced_store != aeg.entry && !aeg.may_source_stb(leak.load, sourced_store)) { continue; } // would be outside of store buffer
        
        // Another approximation of is_ancestor()
        if (aeg.lookup(sourced_store).stores_in > aeg.lookup(leak.bypassed_store).stores_in) {
            continue;
        }
        
        z3_scope;

        const aeg::Node& sourced_store_node = aeg.lookup(sourced_store);
        if (!sourced_store_node.may_write()) { continue; }
        solver.add(sourced_store_node.write, "sourced_store.write");
        
        std::cerr << "run_sourced_store\n";
        
        leak.sourced_store = sourced_store;
        const z3::expr same_addr = aeg::Node::same_addr(sourced_store_node, aeg.lookup(leak.load));
        solver.add(same_addr, "load.addr == sourced_store.addr");
        solver.add(aeg.rfx_exists(sourced_store, leak.load), "load -rfx-> sourced_store");
        
        const auto action = util::push(actions, util::to_string("sourced ", sourced_store));
        
        if (solver.check() == z3::sat) {
            const auto edge = push_edge(EdgeRef {
                .src = leak.load,
                .dst = sourced_store,
                .kind = aeg::Edge::RFX,
            });
            output_execution(leak);
        }
    }
}


void Detector::precompute_rf(NodeRef load) {
    // TODO: only use partial order, not ref2order
    
    std::cerr << "precomputing rf " << load << "\n";
    
    auto& out = rf[load];
    
    if (aeg.exits.find(load) != aeg.exits.end()) { return; }
    const aeg::Node& node = aeg.lookup(load);
    if (!node.may_read()) { return; }
    
#define FILTER_USING_ORDER 1
    
#if FILTER_USING_ORDER
    NodeRefVec order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    
    std::unordered_map<NodeRef, NodeRefVec::const_iterator> ref2order;
    for (auto it = order.begin(); it != order.end(); ++it) {
        ref2order.emplace(*it, it);
    }
#endif
    
    assert(alias_mode.transient);

    NodeRefVec todo;
    util::copy(aeg.po.po.rev.at(load), std::back_inserter(todo));
    NodeRefSet seen;
    while (!todo.empty()) {
        const NodeRef ref = todo.back();
        todo.pop_back();
        if (!seen.insert(ref).second) { continue; }
        
#if FILTER_USING_ORDER
        if (ref2order.at(ref) > ref2order.at(load)) {
            std::cerr << "skipping rf " << ref << " " << load << "\n";
            continue;
            
        }
        
        if (!partial_order(ref, load)) {
            std::cerr << "skipping rf due to partial order\n";
            continue;
        }
#endif
        
#if 0
        if (stb_size) {
            if (aeg.lookup(ref).stores_in > aeg.lookup(load).stores_in + *stb_size) {
                /* must exceed to store buffer size */
                std::cerr << "skipping rf due to stb size\n";
                continue;
            }
        }
        // TODO: this isn't correct.
#endif
        
        if (aeg.lookup(ref).may_write()) {
            switch (aeg.check_alias(load, ref)) {
                case llvm::NoAlias: break;
                    
                case llvm::MayAlias:
                case llvm::MustAlias:
                    out.emplace(ref, mem(load)[node.get_memory_address()] == ctx().int_val((unsigned) ref));
                    break;
                    
                default: std::abort();
            }
        }

        util::copy(aeg.po.po.rev.at(ref), std::back_inserter(todo));
        
        // how to check when 
    }
    
}

const Detector::Sources& Detector::rf_sources(NodeRef load) {
    rf_source_count[load]++;
    std::cerr << rf_source_count << "\n";
    
    auto it = rf.find(load);
    if (it == rf.end()) {
        precompute_rf(load);
        it = rf.find(load);
    }
    assert(it != rf.end());
    return it->second;
}

void Detector::rf_sources(NodeRef load, Sources&& sources) {
    rf[load] = sources;
}

}
