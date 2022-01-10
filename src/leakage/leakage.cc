#include <fstream>
#include <sys/mman.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <system_error>

#include <gperftools/profiler.h>

#include "aeg/aeg.h"
#include "util/timer.h"
#include "util/hash.h"
#include "util/llvm.h"
#include "cfg/expanded.h"
#include "util/output.h"
#include "util/iterator.h"
#include "leakage/leakage.h"
#include "mon/client.h"
#include "mon/proto.h"
#include "util/algorithm.h"
#include "leakage/spectre-v1.h"
#include "leakage/spectre-v4.h"
#include "util/protobuf.h"
#include "leakage/proto.h"
#include "util/sem.h"

extern Transmitters transmitters;


namespace aeg {

void AEG::leakage(z3::solver& solver, TransmitterOutputIt out) {
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
    
    util::copy(detector->get_transmitters(), out);
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
            using ::operator<<;
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
    z3::model model = solver.get_model();
    const z3::eval eval {z3::model(model, ctx(), z3::model::translate())};
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

Detector::Detector(aeg::AEG& aeg, z3::solver& solver):
aeg(aeg),
solver(local_ctx, solver, z3::solver::translate()),
alias_solver(ctx()),
init_mem(z3::const_array(ctx().int_sort(), ctx().int_val(static_cast<unsigned>(aeg.entry)))),
partial_order(aeg.po)
{}

Detector::Mems Detector::get_mems(const NodeRefSet& set) {
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

void Detector::traceback_rf(NodeRef load, aeg::ExecMode exec_mode, std::function<void (NodeRef, CheckMode)> func, CheckMode mode) {
    // Sources new_sources;
    const auto& stores = rf_sources(load);
    if (mode == CheckMode::SLOW) {
        logv(1, __FUNCTION__ << ": tracing back " << stores.size() << " stores\n");
    }
    for (const auto& store_pair : stores) {
        const NodeRef store = store_pair.first;
        
        assert(exec_window.contains(load));
        
        switch (exec_mode) {
            case aeg::ExecMode::ARCH:
            case aeg::ExecMode::EXEC:
                if (!exec_window.contains(store)) { continue; }
                break;
            case aeg::ExecMode::TRANS:
                if (!trans_window.contains(store)) { continue; }
                break;
            default: std::abort();
        }
        
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
            solver_add(cond, desc.c_str());
            solver_add(aeg.lookup(store).exec(exec_mode));
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

void Detector::traceback(NodeRef load, aeg::ExecMode exec_mode, std::function<void (NodeRef, CheckMode)> func, CheckMode mode) {
    const aeg::Node& load_node = aeg.lookup(load);
    
    if (traceback_depth == max_traceback) {
        if (mode == CheckMode::SLOW) {
            std::cerr << "backtracking: max traceback depth (" << max_traceback << ")\n";
        }
        return;
    }
    
    // TODO: POSSIBLY UNNECESSARY SCOPE: there is no forking here
    z3_cond_scope;
    if (mode == CheckMode::SLOW) {
        solver.add(translate(load_node.exec() && load_node.read), util::to_string(load, ".read").c_str());
        if (solver_check() == z3::unsat) { return; }
    }
    
    const auto inc_depth = util::inc_scope(traceback_depth);
    
    // traceback via rf.data
    // TODO: This shouldn't necessarily always be TRANS.
    traceback_rf(load, exec_mode, [&] (const NodeRef store, CheckMode mode) {
        traceback_edge(aeg::Edge::DATA, store, func, mode);
    }, mode);
    
    // traceback via addr
    traceback_edge(aeg::Edge::ADDR, load, func, mode);
}

void Detector::for_one_transmitter(NodeRef transmitter, std::function<void (NodeRef, CheckMode)> func, bool priv) {
    rf.clear();
    
    {
        logv(1, "windows ");
        Timer timer;
        // MAKE EXEC WINDOW
        {
            std::vector<CFG::FuncID> func_id_pfx; // function ID prefix
            exec_window.clear();
            exec_notwindow.clear();
            aeg.for_each_pred_in_window(transmitter, window_size, [&] (NodeRef ref) {
                exec_window.insert(ref);
                const auto& po_node = aeg.po.lookup(ref);
                {
                    std::vector<CFG::FuncID> new_func_id_pfx;
                    util::shared_prefix(func_id_pfx, po_node.id->func, std::back_inserter(new_func_id_pfx));
                    func_id_pfx = std::move(new_func_id_pfx);
                }
            }, [&] (NodeRef ref) {
                exec_notwindow.insert(ref);
            });
            
            // TODO: This should be removed, since the 'pruning paths' step will result in an empty window anyway.
            const bool reaches_main_function = func_id_pfx.empty();
            if (reverse_function_order && !reaches_main_function) {
                logv(2, "skipping transmitter " << transmitter << " because it doesn't reach main function\n");
                return;
            }
            
            /* pruning paths */
            auto new_exec_window = aeg.po.prune_exec_window(exec_window);
            logv(2, "pruned " << (exec_window.size() - new_exec_window.size()) << " transmitters that don't reach main function\n");
            exec_window = std::move(new_exec_window);
            for (NodeRef ref : aeg.po.reverse_postorder()) {
                if (!exec_window.contains(ref)) {
                    exec_notwindow.insert(ref);
                }
            }
            
            mems = get_mems(exec_window);
        }
        
        // MAKE TRANS WINDOW
        {
            trans_window.clear();
            trans_notwindow.clear();
            aeg.for_each_pred_in_window(transmitter, spec_depth, [&] (NodeRef ref) {
                trans_window.insert(ref);
            }, [&] (NodeRef ref) {
                trans_notwindow.insert(ref);
            });
        }
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
        z3::model window_model {ctx()};
        z3::expr F = ctx().bool_val(false);
        z3::expr zero = ctx().int_val(0);
        const auto nullify = [&] (const z3::expr& e, z3::expr& repl) {
            if (e.is_const()) {
                z3::func_decl decl = e.decl();
                window_model.add_const_interp(decl, repl);
            }
        };
        const auto nullify_bool = [&] (const z3::expr& e) {
            nullify(e, F);
        };
        const auto nullify_int = [&] (const z3::expr& e) {
            nullify(e, zero);
        };
        
        for (NodeRef ref : exec_notwindow) {
            assert(!aeg.exits.contains(ref));
            const auto& node = aeg.lookup(ref);
            vec.push_back(!node.exec());
            nullify_bool(node.arch);
            nullify_bool(node.trans);
            nullify_bool(node.read);
            nullify_bool(node.write);
            nullify_bool(node.xsread);
            nullify_bool(node.xswrite);
            if (node.xsaccess_order) {
                nullify_int(*node.xsaccess_order);
            }
        }
        for (NodeRef ref : trans_notwindow) {
            if (!exec_notwindow.contains(ref)) {
                const auto& node = aeg.lookup(ref);
                vec.push_back(!node.trans);
                nullify_bool(node.trans);
            }
        }
        
        /* invalidate edges too */
        {
            aeg.for_each_edge([&] (NodeRef src, NodeRef dst, const aeg::Edge& e) {
                if (!(exec_window.contains(src) && exec_window.contains(dst))) {
                    // invalidate edge
                    nullify_bool(e.exists);
                }
            });
        }
        
        if (priv) {
            Timer timer;
            logv(1, "translating to window...\n");
            z3::solver new_solver {ctx()};
            for (z3::expr old_assertion : solver.assertions()) {
                z3::expr new_assertion = window_model.eval(old_assertion);
                new_assertion = new_assertion.simplify();
                assert(!new_assertion.is_false());
                if (!new_assertion.is_true()) {
                    new_solver.add(new_assertion);
                }
            }
            logv(1, "translated to window in " << timer.get_str() << "\n");
            solver = new_solver;
        }
        
    }
    
    logv(0, __FUNCTION__ << ": adding window constraints\n");
    std::optional<Timer> timer_opt = Timer();
    
    std::optional<z3::scope> scope;
    if (!priv) {
        scope.emplace(solver);
    }
    
    for (const z3::expr& e : vec) { solver.add(translate(e)); }
    logv(0, __FUNCTION__ << ": added window constraints in " << timer_opt->get_str() << "\n");
    timer_opt = std::nullopt;
    
    if (solver_check() != z3::unsat) {
        for (const auto& assertion : aeg.assert_xsaccess_order(exec_window)) {
            solver.add(translate(assertion.first), assertion.second.c_str());
        }
        
        try {
            func(transmitter, CheckMode::SLOW);
        } catch (const next_transmitter& e) {
            // continue
        }
    } else {
        logv(1, "skipping transmitter\n");
        logv(1, "access: " << util::to_string(transmitter_node.access()) << "\n");
        logv(1, "trans: " << util::to_string(transmitter_node.trans) << "\n");
        dbg::append_core(solver, "skipped transmitter");
    }
}

template <class OutputIt>
OutputIt Detector::for_new_transmitter(NodeRef transmitter, std::function<void (NodeRef, CheckMode)> func, OutputIt out) {
    
    char *path;
    if (::asprintf(&path, "%s/tmp/lkg.XXXXXX", output_dir.c_str()) < 0) {
        throw std::system_error(errno, std::generic_category(), "asprintf");
    }
    int fd = ::mkstemp(path);
    std::free(path);
    if (fd < 0) {
        throw std::system_error(errno, std::generic_category(), "mkstemp");
    }
    
    ::signal(SIGSEGV, SIG_DFL);
    ::signal(SIGABRT, SIG_DFL);
    
    const pid_t pid = ::fork();
    if (pid < 0) {
        std::perror("fork");
        std::abort();
    } else if (pid == 0) {
        
        leaks.clear();
        transmitters.clear();
        
        if (semid >= 0) {
            logv(1, "waiting on semaphore...\n");
            semutil::acquire(semid);
            logv(1, "starting\n");
        }
        
        Timer timer;
        
        for_one_transmitter(transmitter, func, true);
        
        if (fast_mode && leaks.size() > 1) {
            std::cerr << "ERROR: num leaks: " << leaks.size() << "\n";
            std::abort();
        }
        
        // write leakage to parent
        for (const auto& leakage_pair : leaks) {
            lkg::LeakageMsg msg;
            for (NodeRef ref : leakage_pair.first.vec) {
                msg.mutable_vec()->Add(ref);
            }
            assert(msg.vec().size() > 1);
            msg.set_transmitter(leakage_pair.first.transmitter);
            msg.set_desc(leakage_pair.second);
            
            if (!proto::write(fd, msg)) {
                std::cerr << "failed to write leakage\n";
                std::abort();
            }
        }
        
        logv(0, "RUNTIME: " << ::getppid() << " " << ::getpid() << " " << cpu_time() << "\n");
        
        std::_Exit(0); // quick exit
    } else {
        *out++ = std::make_pair(pid, Child {.ref = transmitter, .fd = fd});
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
                if (::lseek(fd, 0, SEEK_SET) < 0) {
                    throw std::system_error(errno, std::generic_category(), "lseek");
                }
                
                std::vector<char> buf;
                if (io::readall(fd, buf) < 0) {
                    std::perror("read");
                    std::abort();
                }
                ::close(fd);
                
                const char *ptr = buf.data();
                while (ptr < buf.data() + buf.size()) {
                    lkg::LeakageMsg msg;
                    uint32_t size = *reinterpret_cast<const uint32_t *>(ptr);
                    ptr += 4;
                    if (!msg.ParseFromArray(ptr, size)) {
                        std::cerr << "bad message\n";
                        std::abort();
                    }
                    ptr += size;
                    
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


void Detector::for_each_transmitter(std::function<void (NodeRef, CheckMode)> func) {
    const aeg::Edge::Kind kind = deps().back().first;
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
    
    // filter out any already-seen transmitters
    {
        std::erase_if(candidate_transmitters, [&] (NodeRef ref) -> bool {
            const auto& node = aeg.lookup(ref);
            if (const llvm::Instruction *I = node.inst->get_inst()) {
                if (::transmitters.contains(I)) {
                    llvm::errs() << "filtered transmitter: " << *I << "\n";
                    return true;
                }
            }
            return false;
        });
    }
    
    
    /* make sure that the AEG constraints are satisfiable.
     * NOTE: I should probably move this to AEG::leakage(). */
#ifndef NDEBUG
    {
        const auto check_res = solver.check();
        if (check_res != z3::sat) {
            std::cerr << __FUNCTION__ << ": AEG constraints unsat!\n";
            std::cerr << solver.unsat_core() << "\n";
            std::abort();
        }
    }
#endif
    
    if (max_parallel > 1) {
        
        for_each_transmitter_parallel_private(candidate_transmitters, func);
        
    } else {
        std::cerr << "using 1 thread\n";
        std::size_t i = 0;
        for (NodeRef transmitter : candidate_transmitters) {
            ++i;
            logv(1, i << "/" << candidate_transmitters.size() << "       " << aeg.po.lookup(transmitter) << "\n");
            if (client) {
                mon::Message msg;
                auto *progress = msg.mutable_func_progress();
                progress->mutable_func()->set_name(aeg.po.function_name());
                const float frac = static_cast<float>(i) / static_cast<float>(candidate_transmitters.size());
                progress->set_frac(frac);
                client.send(msg);
            }
            
            for_one_transmitter(transmitter, func, false);
        }
    }
}

NodeRefSet Detector::reachable_r(const NodeRefSet& window, NodeRef init) const {
    NodeRefVec todo = {init};
    NodeRefSet seen;
    while (!todo.empty()) {
        const NodeRef ref = todo.back();
        todo.pop_back();
        if (!window.contains(ref)) { continue; }
        if (!seen.insert(ref).second) { continue; }
        util::copy(aeg.po.po.rev.at(ref), std::back_inserter(todo));
    }
    return seen;
}

std::unordered_map<NodeRef, z3::expr> Detector::precompute_rf_one(NodeRef load, const NodeRefSet& window) {
    const auto& load_node = aeg.lookup(load);
    z3::expr no = ctx().bool_val(true);
    std::unordered_map<NodeRef, z3::expr> yesses;
    for (NodeRef store : aeg.po.postorder()) {
        if (!window.contains(store)) { continue; }
        const auto& store_node = aeg.lookup(store);
        if (!store_node.may_write()) { continue; }
        
        const z3::expr write = store_node.exec() && store_node.write && store_node.same_addr(load_node);
        yesses.emplace(store, no && write);
        
        no = no && !write;
    }
    
    return yesses;
}

void Detector::precompute_rf(NodeRef load) {
    logv(1, "precomputing rf " << load << "\n");
    Timer timer;
    
    auto& out = rf[load];
    
    if (aeg.exits.contains(load)) { return; }
    const aeg::Node& node = aeg.lookup(load);
    if (!node.may_read()) { return; }
    
    assert(alias_mode.transient);
    
    const NodeRefSet window = reachable_r(exec_window, load);
    const auto mem = precompute_rf_one(load, window);
    
    for (const NodeRef ref : window) {
        const auto& store_node = aeg.lookup(ref);
        if (!store_node.may_write()) { continue; }
        
        /* make sure that types agree */
        {
            /* either both pointers or neither pointers */
            const auto& load_node = aeg.lookup(load);
            const auto *load_op = load_node.get_memory_address_pair().first;
            if (const auto *store_inst = dynamic_cast<const MemoryInst *>(store_node.inst.get())) {
                const auto *store_op = store_inst->get_memory_operand();
                
                auto *load_type = load_op->getType()->getPointerElementType();
                assert(load_type == load_node.inst->get_inst()->getType());
                auto *store_type = store_op->getType()->getPointerElementType();
                
                if (load_type->isPointerTy() != store_type->isPointerTy()) {
                    continue;
                }
                
                /* check if type sizes differ */
                llvm::DataLayout DL(store_inst->get_inst()->getModule());
                if (DL.getTypeSizeInBits(load_type) != DL.getTypeSizeInBits(store_type)) {
                    continue;
                }
            }
        }
        
        
        
        /* check if this store occurs before AllocaInst is allocated */
        {
            const auto& load_node = aeg.po.lookup(load);
            if (const auto *AI = llvm::dyn_cast<llvm::AllocaInst>(aeg.lookup(load).get_memory_address_pair().first)) {
                const auto ai_refs = load_node.refs.at(AI);
                if (ai_refs.size() == 1) {
                    const auto ai_ref = *ai_refs.begin();
                    if (ref < ai_ref) {
                        continue; // can't possibly alias
                    }
                }
            }
        }
        
        if (aeg.lookup(ref).may_write()) {
            switch (aeg.compute_alias(load, ref)) {
                case llvm::NoAlias: break;
                    
                case llvm::MayAlias:
                case llvm::MustAlias:
                    /* necessary condition: the two pointers must alias */
                    //out.emplace(ref, mems.at(load)[node.get_memory_address()] == ctx().int_val((unsigned) ref));
                    // out.emplace(ref, mem(load)[node.get_memory_address()] == ctx().int_val((unsigned) ref));
                    out.emplace(ref, mem.at(ref));
                    break;
                    
                default: std::abort();
            }
        }
    }
    
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
    
    solver.add(translate(edge), desc(util::to_string(kind)).c_str());
    
    const auto& src_node = aeg.lookup(src);
    const auto& dst_node = aeg.lookup(dst);
    solver.add(translate(z3::implies(src_node.trans, dst_node.trans)), desc("trans->trans").c_str());
    solver.add(translate(z3::implies(dst_node.arch, src_node.arch)), desc("arch<-arch").c_str());
    
    if (aeg.po.same_basic_block(src, dst)) {
        NodeRef ref = src;
        while (ref != dst) {
            solver.add(translate(aeg.lookup(ref).exec()));
            ref = *aeg.po.po.fwd.at(ref).begin();
        }
    }
}




void Detector::traceback_deps(NodeRef from_ref, std::function<void (const NodeRefVec&, CheckMode)> func, CheckMode mode) {
    NodeRefVec vec;
    DepVec deps;
    if (custom_deps.empty()) {
        deps = this->deps();
    } else {
        deps = custom_deps;
    }
    traceback_deps_rec(deps.rbegin(), deps.rend(), vec, from_ref, func, mode);
}

void Detector::traceback_deps_rec(DepIt it, DepIt end, NodeRefVec& vec, NodeRef from_ref,
                                  std::function<void (const NodeRefVec&, CheckMode)> func, CheckMode mode) {
    const auto push_ref = util::push(vec, from_ref);
    
    if (mode == CheckMode::SLOW) {
        using output::operator<<;
        logv(1, __FUNCTION__ << ": " << vec << "\n");
        
        // lookahead
        if (use_lookahead && !lookahead([&] () {
            traceback_deps_rec(it, end, vec, from_ref, func, CheckMode::FAST);
        })) {
            return;
        }
    }
    
    // check if done (all dependencies found)
    if (it == end) {
        if (mode == CheckMode::SLOW) {
            logv(1, __FUNCTION__ << ": all dependencies found\n");
            if (solver_check() == z3::unsat) {
                logv(1, __FUNCTION__ << ":" << __LINE__ << ": backtrack: unsat\n");
                dbg::append_core(solver, "all dependencies found");
                return;
            }
        }
        func(vec, mode);
        return;
    }
    
    // try committing load
    {
        const aeg::Edge::Kind dep_kind = it->first;
        const aeg::ExecMode dep_src_mode = it->second;
        const auto deps = aeg.get_nodes(Direction::IN, from_ref, dep_kind);
        
        if (deps.empty()) {
            goto label;
        }
        
        if (mode == CheckMode::SLOW) {
            if (solver_check() == z3::unsat) {
                logv(1, __FUNCTION__ << ":" << __LINE__ << ": backtrack: unsat\n");
                dbg::append_core(solver, "committing load");
                return;
            }
            logv(1, "trying to commit " << from_ref << " (" << deps.size() << " deps)\n");
        }
        
        for (const auto& dep : deps) {
            const NodeRef to_ref = dep.first;
            if (!check_edge(to_ref, from_ref)) {
                continue;
            }
            
            z3_cond_scope;
            
            if (mode == CheckMode::SLOW) {
                assert_edge(to_ref, from_ref, dep.second, dep_kind);
                solver.add(translate(aeg.lookup(to_ref).exec(dep_src_mode)), util::to_string(to_ref, " ", dep_kind, " ", from_ref, " ", dep_src_mode).c_str());
            }
            
            const auto push_edge = util::push(flag_edges, EdgeRef {
                .src = to_ref,
                .dst = from_ref,
                .kind = dep_kind
            });
            
            const std::string desc = util::to_string(to_ref, "-", dep_kind, "->", from_ref);
            const auto push_action = util::push(actions, desc);
            
            if (mode == CheckMode::SLOW) {
                logv(1, __FUNCTION__ << ": committed " << desc << "\n");
            }
            
            traceback_deps_rec(std::next(it), end, vec, to_ref, func, mode);
        }
    }
    
label:
    
    /* traceback
     * NOTE: only if it's not the universal transmitter.
     */
    assert(!vec.empty());
    if (from_ref != vec.front()) {
        traceback(from_ref, aeg::ExecMode::TRANS, [&] (NodeRef to_ref, CheckMode mode) {
            if (mode == CheckMode::SLOW) {
                logv(1, "traceback " << to_ref << "-TB->" << from_ref << "\n");
            }
            traceback_deps_rec(it, end, vec, to_ref, func, mode);
        }, mode);
    }
}


z3::check_result Detector::solver_check(bool allow_unknown) {
    z3::check_result res;
    Stopwatch timer;
    timer.start();
    
    const auto timeout = get_timeout();
    if (allow_unknown && timeout) {
        const unsigned timeout2 = static_cast<unsigned>(std::ceil(*timeout * 1000));
        logv(2, __FUNCTION__ << ": checking with time limit " << timeout2 << "ms\n");
        res = z3::check_timeout(solver, timeout2);
    } else {
        do {
            logv(2, __FUNCTION__ << ": checking with no time limit\n");
            res = solver.check();
        } while (res == z3::unknown && solver.reason_unknown() == "canceled");
    }
    
    timer.stop();
    const auto duration = timer.get();
    if (res != z3::unknown) {
        set_timeout(res, duration);
    }
    logv(2, __FUNCTION__ << ": got " << util::to_string(res) << " in " << static_cast<unsigned>(duration * 1000) << "ms\n");
    switch (res) {
        case z3::sat:
            ++check_stats.sat;
            break;
        case z3::unsat:
            ++check_stats.unsat;
            break;
        case z3::unknown:
            ++check_stats.unknown;
            break;
    }
    return res;
}

template <typename OS>
inline OS& operator<<(OS& os, const Detector::CheckStats& stats) {
    const auto frac = [&] (unsigned n) -> std::string {
        if (stats.total() == 0) { return "0%"; }
        std::stringstream ss;
        ss << n * 100 / stats.total() << "%";
        return ss.str();
    };
    os << "sat: " << frac(stats.sat) << ", unsat: " << frac(stats.unsat) << ", unknown: " << frac(stats.unknown);
    return os;
}

Detector::~Detector() {
    logv(1, "stats: " << check_stats << "\n");
}

}
