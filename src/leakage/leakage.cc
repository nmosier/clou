#include <fstream>

#include <gperftools/profiler.h>

#include "aeg/aeg.h"
#include "timer.h"
#include "fork_work_queue.h"
#include "hash.h"
#include "util/llvm.h"
#include "cfg/expanded.h"
#include "util/output.h"
#include "util/iterator.h"
#include "leakage.h"
#include "mon/client.h"
#include "mon/proto.h"
#include "util/algorithm.h"
#include "leakage/spectre-v1.h"
#include "leakage/spectre-v4.h"

namespace aeg {

unsigned AEG::leakage(Solver& solver) {
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

z3::context& Detector::ctx() { return aeg.context.context; }


/* LEAKAGE DETECTOR METHODS */

Detector::Detector(aeg::AEG& aeg, Solver& solver): aeg(aeg), solver(solver), init_mem(z3::const_array(ctx().int_sort(), ctx().int_val(static_cast<unsigned>(aeg.entry)))), mems(get_mems()), partial_order(aeg.po) {}

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

    Mems ins;
    Mems outs = {{aeg.entry, init_mem}};
    for (const NodeRef cur : po.reverse_postorder()) {
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
    for (const NodeRef ref : aeg.po.reverse_postorder()) {
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

Detector::Mems Detector::get_mems1(const NodeRefSet& set) {
    Mems ins;
    Mems outs;
    z3::expr mem = init_mem;
    for (const NodeRef ref : aeg.po.reverse_postorder()) {
        if (ref == aeg.entry || set.find(ref) == set.end()) { continue; }
        const aeg::Node& node = aeg.lookup(ref);
        
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
#if 1
        assert(window.contains(load));
        if (!exec_window.contains(store)) { continue; }
#endif
        
        z3_cond_scope;
        const std::string desc = util::to_string(store, " -rf-> ", load);
        if (mode == CheckMode::SLOW) {
            const z3::expr& cond = store_pair.second;
            solver.add(cond, desc.c_str());
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
        if (!check_edge(edge.first, ref)) { continue; }
        z3_cond_scope;
        if (mode == CheckMode::SLOW) {
            assert_edge(edge.first, ref, edge.second, kind);
        }
        const auto action = util::push(actions, util::to_string(edge.first, " -", kind, "-> ", ref));
        func(edge.first, mode);
    }
}

void Detector::traceback(NodeRef load, std::function<void (NodeRef, CheckMode)> func, CheckMode mode) {
    const aeg::Node& load_node = aeg.lookup(load);
    
    if (traceback_depth == max_traceback) {
        if (mode == CheckMode::SLOW) {
            std::cerr << "backtracking: max traceback depth (" << max_traceback << ")\n";
        }
        return;
    }
    
    z3_cond_scope;
    if (mode == CheckMode::SLOW) {
        solver.add(load_node.exec() && load_node.read, util::to_string(load, ".read").c_str());
        if (solver.check() == z3::unsat) { return; }
    }
    
    const auto inc_depth = util::inc_scope(traceback_depth);
    
    // traceback via rf.data
    traceback_rf(load, [&] (const NodeRef store, CheckMode mode) {
        traceback_edge(aeg::Edge::DATA, store, func, mode);
    }, mode);
    
    // traceback via addr
    traceback_edge(aeg::Edge::ADDR, load, func, mode);
}

void Detector::for_each_transmitter(aeg::Edge::Kind kind, std::function<void (NodeRef, CheckMode)> func) {
    NodeRefSet candidate_transmitters;
    {
        z3::solver solver {ctx()};
        aeg.for_each_edge(kind, [&] (NodeRef, NodeRef ref, const aeg::Edge&) {
            const aeg::Node& node = aeg.lookup(ref);
            z3::expr_vector vec(ctx());
            vec.push_back(node.trans);
            vec.push_back(node.access());
            if (solver.check(vec) != z3::unsat) {
                candidate_transmitters.insert(ref);
            }
        });
    }
    
    std::size_t i = 0;
    for (NodeRef transmitter : candidate_transmitters) {
        ++i;
        logv(1, i << "/" << candidate_transmitters.size() << "\n");
        
        {
            logv(1, "windows ");
            Timer timer;
            // MAKE EXEC WINDOW
            {
                exec_window.clear();
                exec_notwindow.clear();
                aeg.for_each_pred_in_window(transmitter, window_size, [&] (NodeRef ref) {
                    exec_window.insert(ref);
                }, [&] (NodeRef ref) {
                    exec_notwindow.insert(ref);
                });
                mems = get_mems1(exec_window);
            }
            
            // MAKE TRANS WINDOW
            {
                trans_window.clear();
                trans_notwindow.clear();
                aeg.for_each_pred_in_window(transmitter, *max_transient_nodes, [&] (NodeRef ref) {
                    trans_window.insert(ref);
                }, [&] (NodeRef ref) {
                    trans_notwindow.insert(ref);
                });
            }
        }

        if (!lookahead([&] () {
            func(transmitter, CheckMode::FAST);
        })) {
            std::cerr << "skipping transmitter: failed lookahead\n";
            continue;
        }
        
        Timer timer;
                
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
        
        const aeg::Node& transmitter_node = aeg.lookup(transmitter);
        
        if (aeg.exits.find(transmitter) != aeg.exits.end()) {
            continue;
        }
        
        z3::expr_vector vec {ctx()};
        
        // require transmitter is access
        vec.push_back(transmitter_node.access());
        
        // require transmitter.trans
        vec.push_back(transmitter_node.trans);
        
        // window size
#if 0
        const auto saved_mems = util::save(mems);
#endif
        {
            for (NodeRef ref : exec_notwindow) {
                vec.push_back(!aeg.lookup(ref).exec());
            }
            for (NodeRef ref : trans_notwindow) {
                if (!exec_notwindow.contains(ref)) {
                    // vec.push_back(!aeg.lookup(ref).trans);
                }
            }
        }
        
        if (solver.check(vec) != z3::unsat) {
            z3_scope;
            solver.add(vec);
            
            try {
                func(transmitter, CheckMode::SLOW);
            } catch (const next_transmitter& e) {
                // continue
            }
        } else {
            std::cerr << "skipping transmitter\n";
            std::cerr << "access: " << transmitter_node.access() << "\n";
            std::cerr << "trans: " << transmitter_node.trans << "\n";
            dbg::append_core(solver);
        }
    }
}

void Detector::precompute_rf(NodeRef load) {
    // TODO: only use partial order, not ref2order
    
    std::cerr << "precomputing rf " << load << "\n";
    Timer timer;
    
    auto& out = rf[load];
    
    if (aeg.exits.find(load) != aeg.exits.end()) { return; }
    const aeg::Node& node = aeg.lookup(load);
    if (!node.may_read()) { return; }
    
#define FILTER_USING_ORDER 0
    
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
            switch (aeg.compute_alias(load, ref)) {
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
    
#if 1
    for (auto it = out.begin(); it != out.end(); ) {
        if (exec_window.contains(it->first)) {
            ++it;
        } else {
            it = out.erase(it);
        }
    }
#endif
    
}

const Detector::Sources& Detector::rf_sources(NodeRef load) {
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


void Detector::assert_edge(NodeRef src, NodeRef dst, const z3::expr& edge, aeg::Edge::Kind kind) {
    const auto desc = [src, dst] (const std::string& name) -> std::string {
        return util::to_string(name, "-", src, "-", dst);
    };
    
    solver.add(edge, desc(util::to_string(kind)));
    
    const auto& src_node = aeg.lookup(src);
    const auto& dst_node = aeg.lookup(dst);
    solver.add(z3::implies(src_node.trans, dst_node.trans), desc("trans->trans").c_str());
    solver.add(z3::implies(dst_node.arch, src_node.arch), desc("arch<-arch").c_str());
}


}
