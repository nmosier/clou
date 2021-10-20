#include <fstream>

#include <gperftools/profiler.h>

#include "aeg/aeg.h"
#include "timer.h"
#include "fork_work_queue.h"
#include "hash.h"
#include "util/llvm.h"
#include "fol.h"
#include "cfg/expanded.h"
#include "leakage/spectre-v1.h"
#include "leakage/spectre-v4.h"
#include "util/output.h"
#include "util/iterator.h"

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

template <typename OutputIt>
void AEG::leakage_rfx(OutputIt out) const {
    if (util::contains(leakage_sources, LeakageSource::ADDR_DST)) {
        /* NOTE: This is actually an over-approximation, since addr-dst xswrite may not be sourced by an architetcural instruction.
         */
        for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
            std::stringstream ss;
            ss << "rfx-" << addr_dst;
            const Node& addr_dst_node = lookup(addr_dst);
            const z3::expr node_cond = addr_edge.exists && (addr_dst_node.xswrite && addr_dst_node.trans);
            const z3::expr rfx_cond = util::any_of(exits.begin(), exits.end(), [&] (NodeRef exit) {
                return rfx_exists(addr_dst, exit);
            }, context.FALSE);
            *out++ = LeakageClause {
                .pred = node_cond && rfx_cond,
                .name = ss.str(),
                .nodes = {addr_dst},
                .edges = {Edge::ADDR, Edge::RFX},
            };
        });
    }
    
#if 1
    switch (leakage_class) {
        case LeakageClass::SPECTRE_V4: {
            for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
                /*
                 * Require at least 2 arch-prior arch stores; require addr_src be load; require addr_dst be xswrite
                 */
              
            const Node& addr_src_node = lookup(addr_src);
                const Node& addr_dst_node = lookup(addr_dst);
                NodeRefVec order;
                po.reverse_postorder(std::back_inserter(order));
                z3::expr_vector vec {addr_src_node.ctx()};
                for (NodeRef ref : order) {
                    if (ref == addr_src) { break; }
                    const Node& node = lookup(ref);
                    if (!node.may_write()) { continue; }
                    z3::expr cond = node.arch && node.write && node.same_addr(addr_src_node) && node.same_xstate(addr_src_node);
                    vec.push_back(cond);
                }
                
                z3::expr exit_cond = context.FALSE;
                for (NodeRef exit : exits) {
                    exit_cond = exit_cond || rfx_exists(addr_dst, exit);
                }
                
                std::stringstream ss;
                ss << "spectrev4-rfx-" << addr_src << "-" << addr_dst;
                *out++ = LeakageClause {
                    .pred = addr_edge.exists && addr_src_node.read && addr_dst_node.xswrite && addr_src_node.trans && addr_dst_node.trans && z3::atleast(vec, 2) && exit_cond,
                    .name = ss.str(),
                    .nodes = {addr_src},
                    .edges = {Edge::RFX},
                };
            });
            break;
        }
            
        case LeakageClass::SPECTRE_V1:
            break; // TODO: need to move other stuff in here
            
        default:
            todo();
            
            
    }
#endif
    
    if (util::contains(leakage_sources, LeakageSource::CTRL_DST)) {
        /* New kind of leakage to cover PHT10 -- branch condition leakage.
         */
        // check for n0 -ADDR-> n1 -DEP-> n2 -CTRL-> n3.
        for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
            for_each_edge(Edge::CTRL, [&] (NodeRef ctrl_src, NodeRef ctrl_dst, const Edge& ctrl_edge) {
                // TODO: extract dependency query to be separate method
                if (ctrl_src == addr_dst || util::contains(dependencies.at(ctrl_src), addr_dst)) {
                    std::stringstream ss;
                    ss << "rfx-br-" << addr_src << "-" << addr_dst << "-" << ctrl_src << "-" << ctrl_dst;
                    const auto& ctrl_dst_node = lookup(ctrl_dst);
                    if (ctrl_dst_node.can_xswrite()) {
                        *out++ = LeakageClause {
                            .pred = addr_edge.exists && ctrl_edge.exists && ctrl_dst_node.trans && ctrl_dst_node.xswrite,
                            .name = ss.str(),
                            .nodes = {addr_src, addr_dst},
                            .edges = {Edge::ADDR, Edge::CTRL, Edge::RFX},
                        };
                    }
                }
            });
        });
    }
}

template <typename OutputIt>
void AEG::leakage_cox(OutputIt out) const {
#if 0
    // TODO: this is disabled for now, since the number of assertions it generates explodes.
    // silent store check
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (node.may_write()) {
            std::stringstream ss;
            ss << "ss-" << ref;
            *out++ = std::make_tuple(node.arch && node.write && !node.xswrite, ss.str());
        }
    }
    
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        
        // check if there's a subsequent write/xswrite to the same address
        // TODO: test using a lambda function with this.
        for (NodeRef write : node_range()) {
            const Node& write_node = lookup(write);
            if (!write_node.may_write() || !write_node.can_xswrite()) { continue; }
            std::stringstream ss;
            ss << "cox-" << addr_dst << "-" << write;
            *out++ = std::make_tuple((Node::xsaccess_order_less(*this)(addr_dst, write) && (addr_dst_node.xswrite && addr_dst_node.trans) && (write_node.write && write_node.xswrite && write_node.exec())), ss.str());
        }
    });
#endif
}

template <typename OutputIt>
void AEG::leakage_frx(OutputIt out) const {
    
    /*
     frx leakage doesn't ever involve trans addr deps anyway. The only way frx leakage can arise is if A -fr-> B but B is not an xswrite, A is not an xsread, or B's xswrite happens before A's xsread.
     This can still be of interest when considering silent stores, etc.
     Consider a read A. It has to be an xsread. Moving along.
     Consider a write B. There will *always* be an incoming fr edge, with either an arch-pred instruction or program top. If it isn't an xswrite (e.g. is a silent store), the corresponding frx edge will be missing.
     Now, consider if the orders are reversed. Haven't covered this case yet.
     */
    
    // NOTE: This is actually the same assertion as before, in leakage_cox2(). So omitting for now.
#if 0
    for (NodeRef write : node_range()) {
        const Node& write_node = lookup(write);
        if (!write_node.may_write()) { continue; }
        std::stringstream ss;
        ss << "frx-" << write;
        *out++ = std::make_tuple(write_node.arch && write_node.write && !write_node.xswrite, ss.str());
    }
#endif
}

z3::expr AEG::leakage_get_same_solution(const LeakageClause& clause, const z3::eval& eval) {
    fol::Context<bool, fol::ConEval> fol_ctx {fol::Logic<bool>(), fol::ConEval(eval), *this};
    std::unordered_map<Edge::Kind, fol::relation<bool, NodeRef, NodeRef>> edge_rels;
    
    z3::expr same_sol = context.TRUE;
    const auto add = [&] (const z3::expr& e) {
        same_sol = same_sol && e;
    };
    
    for (Edge::Kind kind : clause.edges) {
        edge_rels.emplace(kind, fol_ctx.edge_rel(kind));
    }
    
    // node constraints
    for (NodeRef ref : clause.nodes) {
        const Node& node = lookup(ref);
        add(eval.equal(node.arch) && eval.equal(node.trans));
    }
    
    // edge constraints
    for (NodeRef node : clause.nodes) {
        const auto node_rel = fol_ctx.singleton(std::make_tuple(node));
        for (const auto& all_edges : edge_rels) {
            const auto edge_kind = all_edges.first;
            const auto edges = fol::restrict_any(all_edges.second, node_rel);
            for (const auto& edge : edges) {
                add(exists(edge_kind, std::get<0>(edge.first), std::get<1>(edge.first)));
            }
        }
    }
    
    return same_sol;
}

unsigned AEG::leakage(z3::solver& solver, unsigned max) {
    std::ofstream leakage_ofs {util::to_string(output_dir, "/leakage.txt")};
    
    switch (leakage_class) {
        case LeakageClass::SPECTRE_V4: {
            std::vector<Leakage_SpectreV4> leaks;
            leakage_spectre_v4(solver, std::back_inserter(leaks));
            // leakage_spectre_v4(solver, std::back_inserter(leaks));
            
            // dump leakage
            for (const auto& leak : leaks) {
                leakage_ofs << leak.store0 << " " << leak.store1 << " " << leak.load2 << " " << leak.access3 << " --";
                leakage_ofs << *lookup(leak.store0).inst << "; " << *lookup(leak.store1).inst << "; " << *lookup(leak.load2).inst << "; " << *lookup(leak.access3).inst
                << "\n";
            }
            
            // print out set of transmitters
            std::unordered_set<const llvm::Instruction *> transmitters;
            for (const auto& leak : leaks) {
                transmitters.insert(lookup(leak.access3).inst->get_inst());
            }
            std::cerr << "transmitters:\n";
            for (const auto transmitter : transmitters) {
                llvm::errs() << *transmitter << "\n";
            }
            
            return 0;
        }
          
#if 1
        case LeakageClass::SPECTRE_V1: {
            std::vector<Leakage_SpectreV1_Classic> leaks;
            leakage_spectre_v1(solver, std::back_inserter(leaks));
            
            // dump leakage
            for (const auto& leak : leaks) {
                leakage_ofs << leak.load0 << " " << leak.load1 << " " << leak.transmitter2 << "--" << *lookup(leak.load0).inst << "; " << *lookup(leak.load1).inst << "; " << *lookup(leak.transmitter2).inst << "\n";
            }
            
            // print set of transmitters
            std::unordered_set<const llvm::Instruction *> transmitters;
            for (const auto& leak : leaks) {
                transmitters.insert(lookup(leak.transmitter2).inst->get_inst());
            }
            std::cerr << "transmitters:\n";
            for (const auto transmitter : transmitters) {
                llvm::errs() << *transmitter << "\n";
            }
            
            return 0;
        }
#endif
            
        default: break;
    }
    
    z3::scope scope {solver};
    ProfilerStart("out/leakage.prof");
    
    unsigned nleaks = 0;
    
    std::vector<LeakageClause> clauses;
    const auto clauses_out = std::back_inserter(clauses);
    using ClauseVec = std::vector<LeakageClause>;
    ClauseVec rfx_clauses, cox_clauses, frx_clauses;
    leakage_rfx(std::back_inserter(rfx_clauses));
    leakage_cox(std::back_inserter(cox_clauses));
    leakage_frx(std::back_inserter(frx_clauses));
    std::copy(rfx_clauses.begin(), rfx_clauses.end(), clauses_out);
    std::copy(cox_clauses.begin(), cox_clauses.end(), clauses_out);
    std::copy(frx_clauses.begin(), frx_clauses.end(), clauses_out);

    std::cerr << "rfx clauses: " << rfx_clauses.size() << "\n";
    std::cerr << "cox clauses: " << cox_clauses.size() << "\n";
    std::cerr << "frx clauses: " << frx_clauses.size() << "\n";
    
    std::map<Leakage, unsigned> leakages;
    
    for (std::size_t i = 0; i < clauses.size(); ++i) {
        z3::scope scope {solver};
        const auto& clause = clauses.at(i);
        solver.add(clause.pred, clause.name.c_str());
        
#if 0
        while (true) {
            const z3::check_result res = solver.check();
            std::cerr << res << "\n";
            if (res == z3::unsat) {
                break;
            }
            
             const z3::eval eval {solver.get_model()};
            {
                std::cerr << i << ": ";
                for (const auto& clause : clauses) {
                    if (eval(clause.pred)) {
                        std::cerr << clause.name << " ";
                    }
                }
                std::cerr << "\n";
                
                std::set<Leakage> new_leakages;
                process_leakage(std::inserter(new_leakages, new_leakages.end()), eval);
                
                // output execution
                std::stringstream dot;
                dot << output_dir << "/leakage-" << i << ".dot";
                
                EdgeSet flag_edges;
                for (const Leakage& lkg : new_leakages) {
                    flag_edges.insert(std::tuple_cat(lkg.com, std::make_tuple(lkg.com_kind)));
                    flag_edges.insert(std::tuple_cat(lkg.comx, std::make_tuple(lkg.comx_kind)));
                }
                output_execution(dot.str(), eval, flag_edges);
                
                std::transform(new_leakages.begin(), new_leakages.end(), std::inserter(leakages, leakages.end()), [i] (const auto& x) { return std::make_pair(x, i); });
            }
            
            // generate distinct solutions for current clause
            const z3::expr same_sol = leakage_get_same_solution(clause, eval);
            solver.add(!same_sol);
        }
        
#else
        
        const z3::check_result res = solver.check();
        std::cerr << res << "\n";
        if (res == z3::sat) {
            const z3::eval eval {solver.get_model()};
            
            std::cerr << i << ": ";
            for (const auto& clause : clauses) {
                if (eval(clause.pred)) {
                    std::cerr << clause.name << " ";
                }
            }
            std::cerr << "\n";
            
            std::set<Leakage> new_leakages;
            process_leakage(std::inserter(new_leakages, new_leakages.end()), eval);
            
            // output execution
            std::stringstream dot;
            dot << output_dir << "/leakage-" << i << ".dot";
            
            EdgeVec flag_edges;
            for (const Leakage& lkg : new_leakages) {
                flag_edges.push_back(std::tuple_cat(lkg.com, std::make_tuple(lkg.com_kind)));
                flag_edges.push_back(std::tuple_cat(lkg.comx, std::make_tuple(lkg.comx_kind)));
            }
            output_execution(dot.str(), eval, flag_edges);
            
            std::transform(new_leakages.begin(), new_leakages.end(), std::inserter(leakages, leakages.end()), [i] (const auto& x) { return std::make_pair(x, i); });
        }
#endif
        
        
    }
    
    std::stringstream path;
    path << output_dir << "/leakage.txt";
    std::ofstream ofs {path.str()};
    for (const auto& pair : leakages) {
        const Leakage& leakage = pair.first;
        ofs << pair.second << " " << leakage.com_kind << " " << leakage.com << " " << leakage.comx << " " << leakage.desc;
        ofs << " --" << leakage.com_kind << " (" << po.lookup(leakage.com.first).v << "; " << po.lookup(leakage.com.second).v << ") (" << po.lookup(leakage.comx.first).v << "; " << po.lookup(leakage.comx.second).v << ")\n";
    }
    
    ProfilerStop();
    
    return nleaks;
}

template <typename OutputIt>
OutputIt AEG::process_leakage(OutputIt out, const z3::eval& eval) {
    fol::Context<bool, fol::ConEval> fol_ctx(fol::Logic<bool>(), fol::ConEval(eval), *this);
    const auto trans = fol_ctx.node_rel(ExecMode::TRANS);
    const auto reads = fol_ctx.node_rel(&Node::read, ExecMode::ARCH);
    const auto writes = fol_ctx.node_rel(&Node::write, ExecMode::ARCH);
    const auto accesses = reads + writes;
    const auto xswrites = fol_ctx.node_rel(&Node::xswrite, ExecMode::EXEC);
    const auto xsreads = fol_ctx.node_rel(&Node::xsread, ExecMode::EXEC);
    const auto entry = fol_ctx.node_rel(Inst::Kind::ENTRY, ExecMode::ARCH);
    const auto exit = fol_ctx.node_rel(Inst::Kind::EXIT, ExecMode::ARCH);
    const auto exec = fol_ctx.node_rel(ExecMode::EXEC);
    
    const auto rf = fol_ctx.edge_rel(Edge::RF);
    const auto co = fol_ctx.edge_rel(Edge::CO);
    const auto fr = fol_ctx.edge_rel(Edge::FR);
    const auto rfx = fol_ctx.edge_rel(Edge::RFX);
    const auto cox = fol_ctx.edge_rel(Edge::COX);
    const auto frx = fol_ctx.edge_rel(Edge::FRX);
    const auto same_addr = fol_ctx.same_addr();
    const auto same_xstate = fol_ctx.same_xstate();
    const auto addr = fol_ctx.edge_rel(Edge::ADDR);
    const auto addr_dsts = fol::element<1>(addr);
    const auto ctrl = fol_ctx.edge_rel(Edge::CTRL);
    const auto ctrl_dsts = fol::element<1>(ctrl);
    auto flags = fol_ctx.none<NodeRef>();
    if (util::contains(leakage_sources, LeakageSource::ADDR_DST)) {
        flags |= addr_dsts;
    }
    if (util::contains(leakage_sources, LeakageSource::CTRL_DST)) {
        flags |= ctrl_dsts;
    }
    flags &= trans;
    
    /* rf/rfx leakage */
    {
        // find read nodes that are rfx successors to flagged xswrite nodes
        // trace back architectural rfs from same xstate
        const auto rfx_flagged = fol::restrict_element<0>(rfx, flags);
        const auto same_xstate_flagged = fol::join(fol::element<0>(rfx_flagged), same_xstate);
        const auto rf_flagged = fol::restrict_element<1>(rf, fol::element<1>(rfx_flagged));
        const auto rf_leakage = fol::restrict_element<0>(rf_flagged, same_xstate_flagged);
        const auto rf_leakage2 = fol_ctx.filter(fol::join2(rfx_flagged, ~rf_leakage), [&] (const auto& t) {
            return lookup(std::get<0>(t)).same_xstate(lookup(std::get<2>(t)));
        });
        
        for (const auto& x : rf_leakage2) {
            const auto ref1 = std::get<0>(x.first);
            const auto ref2 = std::get<1>(x.first);
            const auto ref3 = std::get<2>(x.first);
            z3::expr cond {context};
            if (lookup(ref2).inst->is_exit()) {
                // rfx implies rf
                cond = z3::implies(rfx_exists(ref1, ref2), rf_exists(ref1, ref2));
            } else {
                // rf implies rfx
                cond = z3::implies(rf_exists(ref3, ref2), rfx_exists(ref3, ref2));
            }
            
            *out++ = {
                .com_kind = Edge::RF,
                .com = {ref3, ref2},
                .comx_kind = Edge::RFX,
                .comx = {ref1, ref2},
                .desc = "rf without rfx",
                .pred = cond,
            };
        }
        
        std::cerr << "rf leakage: " << rf_leakage2 << "\n";
    }
    
    // co
    {
        // silent stores
        {
            const auto silent_stores = writes - xswrites;
            const auto impacted_cos = fol::restrict_element<0>(co, silent_stores) + fol::restrict_element<1>(co, silent_stores);
            for (const auto& edge : impacted_cos) {
                const NodeRef ref1 = std::get<0>(edge.first);
                const NodeRef ref2 = std::get<1>(edge.first);
                const auto cond = [&] (NodeRef ref) -> z3::expr {
                    const Node& node = lookup(ref);
                    return z3::implies(node.write && node.arch, node.xswrite);
                };
                *out++ = {
                    .com_kind = Edge::CO,
                    .com = {ref1, ref2},
                    .comx_kind = Edge::CO, // really should be 'none'
                    .comx = {0, 0},
                    .desc = "silent store",
                    .pred = cond(ref1) && cond(ref2),
                };
            }
        }
        
        // transient
        {
            const auto co_u = co - fol::join(co, co); // unit co
            const auto cox_u = cox - fol::join(cox, cox); // unit cox
            const auto mismatched_co = co_u - (rfx & cox);
            const auto mismatched_co_rfx = mismatched_co - rfx;
            const auto mismatched_co_cox = mismatched_co - cox;
            const auto mismatched_co_rfx_dsts = fol::element<1>(mismatched_co_rfx);
            const auto rfx_flagged = fol::restrict_element<0>(rfx, flags);
            const auto cox_u_flagged = fol::restrict_element<0>(cox_u, flags);
            const auto co_rfx_triples = fol::join2(mismatched_co_rfx, ~rfx_flagged);
            const auto mismatched_co_cox_dsts = fol::element<1>(mismatched_co_cox);
            const auto co_cox_triples = fol::join2(mismatched_co_cox, ~cox_u_flagged);
            
            const auto f = [&] (const auto& co_comx_triples, Edge::Kind kind) {
                for (const auto& triple : co_comx_triples) {
                    const NodeRef co_src = std::get<0>(triple.first);
                    const NodeRef dst = std::get<1>(triple.first);
                    const NodeRef comx_src = std::get<2>(triple.first);
                    const z3::expr cond = z3::implies(co_exists(co_src, dst), exists(kind, co_src, dst));
                    *out++ = {
                        .com_kind = Edge::CO,
                        .com = {co_src, dst},
                        .comx_kind = kind,
                        .comx = {comx_src, dst},
                        .desc = std::string("co without ") + util::to_string(kind),
                        .pred = cond,
                    };
                }
            };
            
            f(co_rfx_triples, Edge::RFX);
            f(co_cox_triples, Edge::COX);
        }
    }
    
    return out;
}


std::string AEG::leakage_get_path(const std::string& name, const NodeRefVec& vec) {
    std::stringstream ss;
    ss << output_dir << "/" << name;
    for (const NodeRef ref : vec) {
        ss << "-" << ref;
    }
    ss << ".dot";
    return ss.str();
}
