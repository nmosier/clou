#include <fstream>
#include <sys/mman.h>

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
#include "util/protobuf.h"
#include "leakage/proto.h"
#include "util/sem.h"

namespace aeg {

void AEG::leakage(Solver& solver, std::vector<const llvm::Instruction *>& transmitters) {
    std::unique_ptr<lkg::Detector> detector;
    
    switch (leakage_class) {
        case LeakageClass::SPECTRE_V4: {
            detector = std::make_unique<lkg::SpectreV4_Detector>(*this, solver);
            break;
        }
            
        case LeakageClass::SPECTRE_V1: {
            switch (spectre_v1_mode.mode) {
                case SpectreV1Mode::Mode::CLASSIC:
                    detector = std::make_unique<lkg::SpectreV1_Classic_Detector>(*this, solver);
                    break;
                case SpectreV1Mode::Mode::BRANCH_PREDICATE:
                    detector = std::make_unique<lkg::SpectreV1_Control_Detector>(*this, solver);
                    break;
                default: std::abort();
            }
            break;
        }
            
        default: std::abort();
    }
    
    detector->run();
    
    std::cerr << detector->get_transmitters().size() << " trasnmitters\n";
    util::copy(detector->get_transmitters(), std::back_inserter(transmitters));
}

}


namespace lkg {

z3::context& Detector::ctx() { return aeg.context.context; }



void Detector::run() {
    run_();
    
    const std::ios::openmode openmode = batch_mode ? (std::ios::out | std::ios::app) : (std::ios::out);
    
    {
        // open file
        const std::string path = util::to_string(output_dir, "/leakage.txt");
        std::ofstream ofs {
            path,
            openmode,
        };
        
        // dump leakage
        if (batch_mode) {
            ofs << "\n" << aeg.function_name() << ": \n";
        }
        for (const auto& leak : leaks) {
            leak.first.print_short(ofs);
            ofs << " : " << leak.second << " --";
            leak.first.print_long(ofs, aeg);
            ofs << "\n";
        }
    }
    
    {
        const std::string path = util::to_string(output_dir, "/transmitters.txt");
        std::ofstream ofs {
            path,
            openmode,
        };
        
        // print out set of transmitters
        for (const auto& leak : leaks) {
            transmitters.insert(aeg.lookup(leak.first.transmitter).inst->get_inst());
        }
        llvm::errs() << "transmitters:\n";
        for (const auto transmitter : transmitters) {
            llvm::errs() << *transmitter << "\n";
            ofs << *transmitter << "\n";
        }
    }
}


void Detector::output_execution(const Leakage& leak) {
    assert(lookahead_tmp);
    
    leaks.emplace_back(leak, std::accumulate(actions.rbegin(), actions.rend(), std::string(), [] (const std::string& a, const std::string& b) -> std::string {
        std::string res = a;
        if (!res.empty()) {
            res += "; ";
        }
        res += b;
        return res;
    }));
    
    std::stringstream ss;
    ss << output_dir << "/" << name();
    for (const NodeRef ref : leak.vec) {
        ss << "-" << ref;
    }
    ss << ".dot";
    const std::string path = ss.str();
    assert(solver.check() == z3::sat);
    const z3::eval eval {solver.get_model()};
    const auto edge = push_edge(EdgeRef {
        .src = leak.transmitter,
        .dst = aeg.exit_con(eval),
        .kind = aeg::Edge::RFX,
    });
    
    // TODO: shouldn't need to do this
    std::vector<std::tuple<NodeRef, NodeRef, aeg::Edge::Kind>> flag_edges_;
    std::transform(flag_edges.begin(), flag_edges.end(), std::back_inserter(flag_edges_), [] (const EdgeRef& e) {
        return std::make_tuple(e.src, e.dst, e.kind);
    });
    
    // add to detector's transmitters list
    {
        const aeg::Node& transmitter_node = aeg.lookup(leak.transmitter);
        transmitters.insert(transmitter_node.inst->get_inst());
    }
    
    if (witness_executions) {
        aeg.output_execution(path, eval, flag_edges_);
    }
    
    if (fast_mode) {
        throw next_transmitter {};
    }
}


void Leakage::print_long(std::ostream& os, const aeg::AEG& aeg) const {
    for (auto it = vec.begin(); it != vec.end(); ++it) {
        if (it != vec.begin()) {
            os << "; ";
        }
        os << *aeg.lookup(*it).inst;
    }
}

void Leakage::print_short(std::ostream& os) const {
    for (auto it = vec.begin(); it != vec.end(); ++it) {
        if (it != vec.begin()) {
            os << " ";
        }
        os << *it;
    }
}


/* LEAKAGE DETECTOR METHODS */

Detector::Detector(aeg::AEG& aeg, Solver& solver): aeg(aeg), solver(solver), alias_solver(ctx()), init_mem(z3::const_array(ctx().int_sort(), ctx().int_val(static_cast<unsigned>(aeg.entry)))), mems(get_mems()), partial_order(aeg.po) {}

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
    if (!use_lookahead) {
        return true;
    }
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
    const auto& stores = rf_sources(load);
    if (mode == CheckMode::SLOW) {
        logv(1, __FUNCTION__ << ": tracing back " << stores.size() << " stores\n");
    }
    for (const auto& store_pair : stores) {
        const NodeRef store = store_pair.first;
#if 1
        assert(exec_window.contains(load));
        if (!exec_window.contains(store)) { continue; }
#endif
        if (mode == CheckMode::SLOW && use_lookahead && !lookahead([&] () {
            func(store, CheckMode::FAST);
        })) {
            if (mode == CheckMode::SLOW) {
                logv(1, __FUNCTION__ << ": skipping: failed lookahead\n");
            }
            continue;
        }
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
        
        if (mode == CheckMode::SLOW && use_lookahead && !lookahead([&] () {
            func(edge.first, CheckMode::FAST);
        })) {
            continue;
        }
        
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

void Detector::for_one_transmitter(NodeRef transmitter, std::function<void (NodeRef, CheckMode)> func) {
    rf.clear();
    
    bool window_changed = false;
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
        
        // TODO: conditionally set this properly
        window_changed = true;
    }
    
    if (use_lookahead && !lookahead([&] () {
        func(transmitter, CheckMode::FAST);
    })) {
        logv(1, __FUNCTION__ << "skipping transmitter: failed lookahead\n");
        return;
    }
    
    Timer timer;
    
    if (transmitters.find(aeg.lookup(transmitter).inst->get_inst()) != transmitters.end()) {
        return;
    }
    
    const auto action = util::push(actions, util::to_string("transmitter ", transmitter));
    
    const aeg::Node& transmitter_node = aeg.lookup(transmitter);
    
    if (aeg.exits.find(transmitter) != aeg.exits.end()) {
        return;
    }
    
    z3::expr_vector vec {ctx()};
    
    // require transmitter is access
    vec.push_back(transmitter_node.access());
    
    // require transmitter.trans
    vec.push_back(transmitter_node.trans);
    
    // window size
    {
        for (NodeRef ref : exec_notwindow) {
            vec.push_back(!aeg.lookup(ref).exec());
        }
        for (NodeRef ref : trans_notwindow) {
            if (!exec_notwindow.contains(ref)) {
                vec.push_back(!aeg.lookup(ref).trans);
            }
        }
    }
    
    logv(0, __FUNCTION__ << ": adding window constraints\n");
    std::optional<Timer> timer_opt = Timer();
    z3_scope;
    for (const z3::expr& e : vec) { solver.add(e); }
    timer_opt = std::nullopt;
    
    if (solver.check() != z3::unsat) {
        aeg.assert_xsaccess_order(exec_window, solver);
        
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

template <class OutputIt>
OutputIt Detector::for_new_transmitter(NodeRef transmitter, std::function<void (NodeRef, CheckMode)> func, OutputIt out) {
    int fds[2];
    io::pipe(fds);
    
    ::signal(SIGSEGV, SIG_DFL);
    ::signal(SIGABRT, SIG_DFL);
    
    const pid_t pid = ::fork();
    if (pid < 0) {
        std::perror("fork");
        std::abort();
    } else if (pid == 0) {
        
        if (semid >= 0) {
            semutil::acquire(semid);
        }
        
        ::close(fds[0]);
        for_one_transmitter(transmitter, func);
        
        // write leakage to parent
        for (const auto& leakage_pair : leaks) {
            lkg::LeakageMsg msg;
            for (NodeRef ref : leakage_pair.first.vec) {
                msg.mutable_vec()->Add(ref);
            }
            assert(msg.vec().size() > 1);
            msg.set_transmitter(leakage_pair.first.transmitter);
            msg.set_desc(leakage_pair.second);
            
            if (!proto::write(fds[1], msg)) {
                std::cerr << "failed to write leakage\n";
                std::abort();
            }
        }
        ::close(fds[1]);
        
        std::_Exit(0); // quick exit
    } else {
        *out++ = std::make_pair(pid, Child {.ref = transmitter, .fd = fds[0]});
        ::close(fds[1]);
        return out;
    }
}

void Detector::for_each_transmitter_parallel_private(NodeRefSet& candidate_transmitters, std::function<void (NodeRef, CheckMode)> func) {

    std::cerr << "using " << max_parallel << " threads\n";
    
    unsigned num_threads = 0;
    std::unordered_map<pid_t, Child> children;
    std::size_t total_candidate_transmitters = candidate_transmitters.size();
    std::size_t i = 0;
    while (true) {
        
        /* spawn new child if possible */
        if (num_threads < max_parallel && !candidate_transmitters.empty()) {
            const auto it = candidate_transmitters.begin();
            const NodeRef transmitter = *it;
            candidate_transmitters.erase(it);
            
            for_new_transmitter(transmitter, func, std::inserter(children, children.end()));
            ++num_threads;
            
            ++i;
            logv(1, i << "/" << total_candidate_transmitters << "\n");
            
            if (client) {
                mon::Message msg;
                auto *progress = msg.mutable_func_progress();
                progress->mutable_func()->set_name(aeg.po.function_name());
                const float frac = static_cast<float>(i) / static_cast<float>(total_candidate_transmitters);
                progress->set_frac(frac);
                client.send(msg);
            }
        }
        
        /* reap dead children if necessary */
        if (num_threads == max_parallel || (num_threads > 0 && candidate_transmitters.empty())) {
            int status;
            pid_t pid;
            while (true) {
                pid = ::wait(&status);
                if (pid < 0 && errno != EINTR) {
                    std::perror("wait");
                    std::abort();
                }
                if (pid >= 0) {
                    break;
                }
            }
            
            const Child& child = children.at(pid);
            if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
                std::cerr << "child aborted or had nonzero exit code: ";
                print_status(std::cerr, status);
                std::cerr << "\n";
                logv(0, "restarting " << child.ref << "\n");
                
                // try to recover
                --i;
                candidate_transmitters.insert(child.ref);
                
            } else {
                
                logv(0, "finished " << child.ref << "\n");
                
                const int fd = children.at(pid).fd;
                
                std::vector<char> buf;
                if (io::readall(fd, buf) < 0) {
                    std::perror("read");
                    std::abort();
                }
                ::close(fd);
                
                std::size_t rem = buf.size();
                const char *ptr = buf.data();
                while (ptr < buf.data() + buf.size()) {
                    lkg::LeakageMsg msg;
                    uint32_t size = *reinterpret_cast<const uint32_t *>(ptr);
                    std::cerr << "message size " << size << "\n";
                    rem -= 4;
                    ptr += 4;
                    if (!msg.ParseFromArray(ptr, size)) {
                        std::cerr << "bad message\n";
                        std::abort();
                    }
                    ptr += size;
                    rem -= size;
                    
                    NodeRefVec vec;
                    util::copy(msg.vec(), std::back_inserter(vec));
                    assert(msg.vec().size() > 1);
                    assert(vec.size() > 1);
                    NodeRef transmitter = msg.transmitter();
                    std::string desc = msg.desc();
                    leaks.emplace_back(Leakage {
                        .vec = vec,
                        .transmitter = transmitter
                    }, desc);
                }
            
            }
            
            --num_threads;
        }
        
        if (num_threads == 0 && candidate_transmitters.empty()) {
            break;
        }
        
        /* update number of live threads */
        client.send_property(aeg.function_name(), "threads", num_threads);
    }
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
    
    /* make sure that the AEG constraints are satisfiable.
     * NOTE: I should probably move this to AEG::leakage(). */
    {
        const auto check_res = solver.check();
        if (check_res != z3::sat) {
            std::cerr << __FUNCTION__ << ": AEG constraints unsat!\n";
            std::cerr << solver.unsat_core() << "\n";
            std::abort();
        }
    }
    
    if (max_parallel > 1) {
        
        for_each_transmitter_parallel_private(candidate_transmitters, func);
        
    } else {
        std::cerr << "using 1 thread\n";
        std::size_t i = 0;
        for (NodeRef transmitter : candidate_transmitters) {
            ++i;
            logv(1, i << "/" << candidate_transmitters.size() << "\n");
            
            if (client) {
                mon::Message msg;
                auto *progress = msg.mutable_func_progress();
                progress->mutable_func()->set_name(aeg.po.function_name());
                const float frac = static_cast<float>(i) / static_cast<float>(candidate_transmitters.size());
                progress->set_frac(frac);
                client.send(msg);
            }
            
            for_one_transmitter(transmitter, func);
        }
    }
}

void Detector::precompute_rf(NodeRef load) {
    // TODO: only use partial order, not ref2order
    
    std::cerr << "precomputing rf " << load << "\n";
    Timer timer;
    
    auto& out = rf[load];
    
    if (aeg.exits.contains(load)) { return; }
    const aeg::Node& node = aeg.lookup(load);
    if (!node.may_read()) { return; }
    
    assert(alias_mode.transient);
    
    NodeRefVec todo;
    util::copy(aeg.po.po.rev.at(load), std::back_inserter(todo));
    NodeRefSet seen;
    while (!todo.empty()) {
        const NodeRef ref = todo.back();
        todo.pop_back();
        if (!seen.insert(ref).second) { continue; }
        if (!exec_window.contains(ref)) { continue; }
        
        if (aeg.lookup(ref).may_write()) {
            switch (aeg.compute_alias(load, ref)) {
                case llvm::NoAlias: break;
                    
                case llvm::MayAlias:
                case llvm::MustAlias:
                    /* necessary condition: the two pointers must alias */
                    out.emplace(ref, mem(load)[node.get_memory_address()] == ctx().int_val((unsigned) ref));
                    break;
                    
                default: std::abort();
            }
        }
        
        util::copy(aeg.po.po.rev.at(ref), std::back_inserter(todo));
    }
    
#if 0
    // NOTE: This is now handled in above while loop.
    // filter by exec window
    for (auto it = out.begin(); it != out.end(); ) {
        if (exec_window.contains(it->first)) {
            ++it;
        } else {
            it = out.erase(it);
        }
    }
#endif
    
#if 1
    // filter by satisfiable aliases
    unsigned filtered = 0;
    for (auto it = out.begin(); it != out.end(); ) {
        bool keep = true;

        // check if alias expression is always false
        if (it->first != aeg.entry) {
            const z3::expr alias = (aeg.lookup(it->first).get_memory_address() == aeg.lookup(load).get_memory_address());
            if (alias.simplify().is_false()) {
                keep = false;
            } else {
                z3::expr_vector vec(ctx());
                vec.push_back(alias);
                if (alias_solver.check(vec) == z3::unsat) {
                    keep = false;
                }
            }
        }
        
        if (keep) {
            ++it;
        } else {
            it = out.erase(it);
            ++filtered;
        }
    }
    logv(1, __FUNCTION__ << ": filtered " << filtered << "\n");
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
    
    solver.add(edge, desc(util::to_string(kind)).c_str());
    
    const auto& src_node = aeg.lookup(src);
    const auto& dst_node = aeg.lookup(dst);
    solver.add(z3::implies(src_node.trans, dst_node.trans), desc("trans->trans").c_str());
    solver.add(z3::implies(dst_node.arch, src_node.arch), desc("arch<-arch").c_str());
}


}
