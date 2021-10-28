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

Detector::Mems Detector::get_mems() {
    z3::context& ctx = this->ctx();
    auto& po = aeg.po;
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));

    const z3::expr init_mem = z3::const_array(ctx.int_sort(), ctx.int_val(static_cast<unsigned>(aeg.entry)));
    Mems ins;
    Mems outs = {{aeg.entry, init_mem}};
    for (const NodeRef cur : order) {
        if (cur == aeg.entry) { continue; }
        const auto& cur_node = aeg.lookup(cur);
        
        const auto tfos = aeg.get_nodes(Direction::IN, cur, aeg::Edge::TFO);
        assert(!tfos.empty());
        
        z3::expr mem {ctx};
        
        auto tfo_it = tfos.begin();
        mem = outs.at(tfo_it->first);
        ++tfo_it;
        mem = std::accumulate(tfo_it, tfos.end(), mem, [&] (const z3::expr& acc, const auto& tfo) -> z3::expr {
            return z3::ite(tfo.second, outs.at(tfo.first), acc);
        });
        
        ins.emplace(cur, mem);
        
        if (cur_node.may_write()) {
            mem = z3::conditional_store(mem, cur_node.get_memory_address(), ctx.int_val(static_cast<unsigned>(cur)), cur_node.write);
        }
        
        outs.emplace(cur, mem);
    }
    
    return ins;
}


Detector::Mems Detector::get_mems_arch() {
    z3::context& ctx = this->ctx();

    NodeRefVec order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    
    const z3::expr init_mem = z3::const_array(ctx.int_sort(), ctx.int_val(static_cast<uint64_t>(aeg.entry)));
    
    Mems ins;
    Mems outs = {{aeg.entry, init_mem}};
    z3::expr mem = init_mem;
    for (const NodeRef ref : order) {
        if (ref == aeg.entry) { continue; }
        const aeg::Node& node = aeg.lookup(ref);
        ins.emplace(ref, mem);
        
        if (node.may_write()) {
            mem = z3::conditional_store(mem, node.get_memory_address(), ctx.int_val(static_cast<uint64_t>(ref)), node.arch && node.write);
        }
        
        outs.emplace(ref, mem);
    }
    
    return ins;
}

Detector::Mems Detector::get_mems_trans() {
    z3::context& ctx = this->ctx();
    
    NodeRefVec order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    
    const z3::expr init_mem = z3::const_array(ctx.int_sort(), ctx.int_val(static_cast<uint64_t>(aeg.entry)));
    
    const auto apply_write = [&] (const NodeRef ref, z3::expr& mem) {
        const aeg::Node& node = aeg.lookup(ref);
        if (!node.may_write()) { return; }
        mem = z3::conditional_store(mem, node.get_memory_address(), ctx.int_val(static_cast<uint64_t>(ref)), node.exec() && node.write);
    };
    
    Mems ins;
    Mems outs = {{aeg.entry, init_mem}};

    for (const NodeRef ref : order) {
        if (ref == aeg.entry) { continue; }
        const NodeRefSet& equivs = aeg.control_equivalents[ref];
        if (equivs.empty()) {
            
            // old fashioned way: merge predecessors using ite
            const auto tfos = aeg.get_nodes(Direction::IN, ref, aeg::Edge::TFO);
            assert(!tfos.empty());
            auto it = tfos.begin();
            z3::expr mem = outs.at(it->first);
            for (++it; it != tfos.end(); ++it) {
                mem = z3::ite(it->second, outs.at(it->first), mem);
            }
            ins.emplace(ref, mem);
            apply_write(ref, mem);
            outs.emplace(ref, mem);

        } else {
            // get closest control-equivalent ancestor and set of intervening nodes
            NodeRefVec todo = {ref};
            NodeRefSet seen;
            std::optional<NodeRef> equiv; // nearest equiv, initially unknown
            while (!todo.empty()) {
                const NodeRef cur = todo.back();
                todo.pop_back();
                
                // check if found control-equivalent node
                if (equivs.find(cur) != equivs.end()) {
                    equiv = cur;
                    continue;
                }
                
                // check if seen
                if (!seen.insert(cur).second) { continue; }
                
                // explore predecessors
                util::copy(aeg.po.po.rev.at(cur), std::back_inserter(todo));
            }
            assert(equiv);
            
            // apply writes
            z3::expr mem = outs.at(*equiv);
            
            for (auto it = order.begin(); *it != ref; ++it) {
                apply_write(*it, mem);
            }
            ins.emplace(ref, mem);
            apply_write(ref, mem);
            outs.emplace(ref, mem);
        }
        
    }
    
    return ins;
}

void Detector::traceback_rf(NodeRef load, std::function<void (NodeRef)> func) {
#if 0
    const z3::expr store_sym = mems.at(load)[aeg.lookup(load).get_memory_address()];
    std::vector<z3::expr> stores_con;
    z3::enumerate(solver, store_sym, std::back_inserter(stores_con));
    
    for (const z3::expr& store_con : stores_con) {
        z3_scope;
        const NodeRef store = store_con.get_numeral_uint();
        solver.add(store_con == store_sym, util::to_string(store, " -rf-> ", load).c_str());
        func(store);
    }
#elif 0
    for (const auto& store_pair : rf[load]) {
        const NodeRef store = store_pair.first;
        const z3::expr& cond = store_pair.second;
        z3_scope;
        solver.add(cond, util::to_string(store, " -rf-> ", load).c_str());
        func(store);
    }
#elif 1
    // Sources new_sources;
    for (const auto& store_pair : rf_sources(load)) {
        const NodeRef store = store_pair.first;
        const z3::expr& cond = store_pair.second;
        z3_scope;
        
        const std::string desc = util::to_string(store, " -rf-> ", load).c_str();
        solver.add(cond, desc.c_str());
        if (solver.check() != z3::sat) { continue; }
        // new_sources.insert(store_pair);
        // TODO: need to separately check if this is due to something else
        func(store);
    }
    // rf_sources(load, std::move(new_sources));
#else
    const aeg::Node& load_node = aeg.lookup(load);
    for (const auto& store_pair : rf_sources(load)) {
        const NodeRef store = store_pair.first;
        const aeg::Node& store_node = aeg.lookup(store);
        
        z3_scope scope;
    }
    
#endif
}


void Detector::traceback(NodeRef load, std::function<void (NodeRef)> func) {
    const aeg::Node& load_node = aeg.lookup(load);
#if 0
    assert(load_node.may_read());
#else
    z3_scope;
    solver.add(load_node.exec() &&  load_node.read, util::to_string(load, ".read").c_str());
    if (solver.check() != z3::sat) { return; }
#endif
    
    if (traceback_depth == max_traceback) {
        return;
    }
    
    const auto inc_depth = util::inc_scope(traceback_depth);
    
    unsigned num_traced_back = 0;
    const auto traceback_edge = [&] (aeg::Edge::Kind kind, NodeRef ref) {
        const auto edges = aeg.get_nodes(Direction::IN, ref, kind);
        for (const auto& edge : edges) {
            z3_scope;
            solver.add(edge.second, util::to_string(edge.first, " -", kind, "-> ", ref).c_str());
            func(edge.first);
        }
        ++num_traced_back;
    };
    
    // traceback via rf.data
    traceback_rf(load, [&] (const NodeRef store) {
        traceback_edge(aeg::Edge::DATA, store);
    });
    
    // traceback via addr
    traceback_edge(aeg::Edge::ADDR, load);
    
    if (num_traced_back == 0) {
        trace("backtrack: no traceback edges\n");
    }
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


void Detector::for_each_transmitter(aeg::Edge::Kind kind, std::function<void (NodeRef)> func) const {
    NodeRefSet candidate_transmitters;
    aeg.for_each_edge(kind, [&] (NodeRef, NodeRef ref, const aeg::Edge&) {
        candidate_transmitters.insert(ref);
    });
    
    std::size_t i = 0;
    for (NodeRef transmitter : candidate_transmitters) {
        std::cerr << ++i << "/" << candidate_transmitters.size() << "\n";
        
        z3_scope;
        const aeg::Node& transmitter_node = aeg.lookup(transmitter);
        
        if (aeg.exits.find(transmitter) != aeg.exits.end()) {
            continue;
        }
        
        // require transmitter is access
        solver.add(transmitter_node.access(), "transmitter.access");
        
        // require transmitter.trans
        solver.add(transmitter_node.trans, "transmitter.trans");
        
        if (solver.check() == z3::sat) {
            try {
                func(transmitter);
            } catch (const next_transmitter& e) {
                // continue
            }
        }
    }
}


void SpectreV1_Detector::run_() {
    for_each_transmitter(deps().back(), [&] (const NodeRef transmitter) {
        run1(transmitter, transmitter);
    });
}

void SpectreV1_Detector::run1(NodeRef transmitter, NodeRef access) {
    std::cerr << __FUNCTION__ << ": transmitter=" << transmitter << " access=" << access << " loads=" << loads << "\n";
    
    if (solver.check() != z3::sat) {
        trace("backtrack: unsat");
        return;
    }
    
    if (loads.size() == deps().size()) {
        z3_eval;
        
        const SpectreV1_Leakage leak = {
            .load0 = loads.at(1),
            .load1 = loads.at(0),
            .transmitter2 = transmitter,
        };
        
        std::cerr << "spectre-v1 leak found\n";
        
        output_execution(leak);
        
        return;
    }
    
    assert(loads.size() < deps().size());
    
    /* try committing load */
    {

        const aeg::Edge::Kind dep_kind = cur_dep();
        const std::string dep_str = util::to_string(dep_kind);
        const auto deps = aeg.get_nodes(Direction::IN, access, dep_kind);
        
        trace("trying to commit %lu (%zu deps)", access, deps.size());

        for (const auto& dep : deps) {
            z3_scope;
            const NodeRef load = dep.first;
            const auto push_load = util::push(loads, load);
            solver.add(dep.second, util::to_string(load, " -", dep_kind, "-> ", access).c_str());
            const auto addr_edge = util::push(flag_edges, EdgeRef {
                .src = load,
                .dst = access,
                .kind = aeg::Edge::ADDR,
            });
                
            std::cerr << __FUNCTION__ << ": committed " << load << " -" << dep_kind << "-> " << access << "\n";
            run1(transmitter, load);
        }
    }
    
    /* traceback */
    if (access != transmitter) {
        traceback(access, [&] (const NodeRef load) {
            std::cerr << name() << ": traceback " << access << " to " << load << "\n";
            run1(transmitter, load);
        });
    }
}


SpectreV1_Classic_Detector::DepVec SpectreV1_Classic_Detector::deps() const {
    return {aeg::Edge::ADDR, aeg::Edge::ADDR};
}

SpectreV1_Control_Detector::DepVec SpectreV1_Control_Detector::deps() const {
    return {aeg::Edge::ADDR, aeg::Edge::CTRL};
}


void SpectreV4_Detector::run_() {
    for_each_transmitter(aeg::Edge::ADDR, [&] (const NodeRef transmitter) {
        std::cerr << "transmitter " << transmitter << "\n";
        leak.transmitter = transmitter;
        run_load(transmitter);
    });
}

void SpectreV4_Detector::run_load(NodeRef access) {
    // bind loads
    {
        const auto addrs = aeg.get_nodes(Direction::IN, access, aeg::Edge::ADDR);
        for (const auto& addr : addrs) {
            z3_scope;
            const NodeRef load = addr.first;
            leak.load = load;
            const auto edge = push_edge({
                .src = load,
                .dst = access,
                .kind = aeg::Edge::ADDR,
            });
            solver.add(addr.second, util::to_string(load, " -addr-> ", access).c_str());
            solver.add(aeg.lookup(load).trans, "load.trans");
            if (solver.check() == z3::sat) {
                run_bypassed_store();
            }
        }
    }
    
    // traceback
    traceback(access, [&] (NodeRef load) {
        std::cerr << "traceback " << load << "\n";
        run_load(load);
    });
}


void SpectreV4_Detector::run_bypassed_store() {
    traceback_rf(leak.load, [&] (const NodeRef bypassed_store) {
        z3_scope;
        leak.bypassed_store = bypassed_store;
        const auto edge = push_edge({
            .src = bypassed_store,
            .dst = leak.load,
            .kind = aeg::Edge::ADDR,
        });
        run_sourced_store();
    });
}


void SpectreV4_Detector::run_sourced_store() {
    for (const NodeRef sourced_store : aeg.node_range()) {
        z3_scope;

        const aeg::Node& sourced_store_node = aeg.lookup(sourced_store);
        if (!sourced_store_node.may_write()) { continue; }
        solver.add(sourced_store_node.write, "sourced_store.write");
        
        std::cerr << "run_sourced_store\n";
        
        leak.sourced_store = sourced_store;
        const z3::expr same_addr = aeg::Node::same_addr(sourced_store_node, aeg.lookup(leak.load));
        solver.add(same_addr, "load.addr == sourced_store.addr");
        solver.add(aeg.rfx_exists(sourced_store, leak.load), "load -rfx-> sourced_store");
        
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
