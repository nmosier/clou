#include <fstream>

#include <gperftools/profiler.h>

#include "aeg.h"
#include "timer.h"
#include "fork_work_queue.h"
#include "hash.h"
#include "llvm-util.h"
#include "fol.h"

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
void AEG::leakage_rfx2(OutputIt out) const {
    /* NOTE: This is actually an over-approximation, since addr-dst xswrite may not be sourced by an architetcural instruction.
     *
     */
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        std::stringstream ss;
        ss << "rfx-" << addr_dst;
        const Node& addr_dst_node = lookup(addr_dst);
        *out++ = std::make_tuple(addr_edge.exists && addr_dst_node.xswrite && addr_dst_node.trans, ss.str());
    });
}

template <typename OutputIt>
void AEG::leakage_cox2(OutputIt out) const {
    // silent store check
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (node.is_write()) {
            std::stringstream ss;
            ss << "ss-" << ref;
            *out++ = std::make_tuple(node.arch && !node.xswrite, ss.str());
        }
    }
    
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        
        // check if there's a subsequent write/xswrite to the same address
        // TODO: test using a lambda function with this.
        for (NodeRef write : node_range()) {
            const Node& write_node = lookup(write);
            if (!write_node.is_write() || write_node.xswrite.is_false()) { continue; }
            std::stringstream ss;
            ss << "cox-" << addr_dst << "-" << write;
            *out++ = std::make_tuple((Node::xsaccess_order_less(*this)(addr_dst, write) && addr_dst_node.xswrite && addr_dst_node.trans && write_node.xswrite && write_node.exec()), ss.str());
        }
    });
}

template <typename OutputIt>
void AEG::leakage_frx2(OutputIt out) const {
    
    /*
     frx leakage doesn't ever involve trans addr deps anyway. The only way frx leakage can arise is if A -fr-> B but B is not an xswrite, A is not an xsread, or B's xswrite happens before A's xsread.
     This can still be of interest when considering silent stores, etc.
     Consider a read A. It has to be an xsread. Moving along.
     Consider a write B. There will *always* be an incoming fr edge, with either an arch-pred instruction or program top. If it isn't an xswrite (e.g. is a silent store), the corresponding frx edge will be missing.
     Now, consider if the orders are reversed. Haven't covered this case yet.
     */
    
    // NOTE: This is actually the same assertion as before, in leakage_cox2(). So omitting for now.
#if 1
    for (NodeRef write : node_range()) {
        const Node& write_node = lookup(write);
        if (!write_node.is_write()) { continue; }
        std::stringstream ss;
        ss << "frx-" << write;
        *out++ = std::make_tuple(write_node.arch && !write_node.xswrite, ss.str());
    }
#endif
}

unsigned AEG::leakage2(z3::solver& solver, unsigned max) {
    ProfilerStart("out/leakage.prof");
    
    solver.push();
    
    unsigned nleaks = 0;
    
    std::vector<std::tuple<z3::expr, std::string>> clauses;
    const auto clauses_out = std::back_inserter(clauses);
    leakage_rfx2(clauses_out);
    leakage_cox2(clauses_out);
    leakage_frx2(clauses_out);
    const z3::expr leakage = std::transform_reduce(clauses.begin(), clauses.end(), context.FALSE, util::logical_or<z3::expr>(), [] (const auto& t) -> z3::expr {
        return std::get<0>(t);
    });
    solver.add(leakage, "leakage");
    
    std::map<Leakage, unsigned> leakages;
    
    for (unsigned i = 0; i < max; ++i) {
        const z3::check_result res = solver.check();
        std::cerr << res << "\n";
        switch (res) {
            case z3::unsat:
                std::cerr << solver.unsat_core() << "\n";
                goto done;
            case z3::sat: {
                const z3::eval eval {solver.get_model()};
                
                std::cerr << i << ": ";
                for (const auto& clause : clauses) {
                    if (eval(std::get<0>(clause))) {
                        std::cerr << std::get<1>(clause) << " ";
                    }
                }
                std::cerr << "\n";
                
                std::set<Leakage> new_leakages;
                process_leakage(std::inserter(new_leakages, new_leakages.end()), eval);
                
                // add constraints
                std::stringstream dot;
                dot << output_dir << "/leakage-" << i << ".dot";
                
                EdgeSet flag_edges;
                for (const Leakage& lkg : new_leakages) {
                    flag_edges.insert(std::tuple_cat(lkg.com, std::make_tuple(lkg.com_kind)));
                    flag_edges.insert(std::tuple_cat(lkg.comx, std::make_tuple(lkg.comx_kind)));
                }
                output_execution(dot.str(), eval, flag_edges);
                
                for (const Leakage& lkg : new_leakages) {
                    std::stringstream ss;
                    ss << Edge::kind_tostr(lkg.com_kind) << "-" << lkg.com << "-" << lkg.comx << "-" << i;
                    solver.add(lkg.pred, ss.str().c_str());
                }
                
                const auto before = leakages.size();
                std::transform(new_leakages.begin(), new_leakages.end(), std::inserter(leakages, leakages.end()), [i] (const auto& x) { return std::make_pair(x, i); });
                const auto after = leakages.size();
                std::cerr << "new leakages: " << after - before << "\n";
                
                
                // TODO: process concrete output to get all leakage
                
                ++nleaks;
                
                break;
            }
            case z3::unknown: throw std::logic_error("unknown");
        }
    }
    
done:
    solver.pop();
    
    std::stringstream path;
    path << output_dir << "/leakage.txt";
    std::ofstream ofs {path.str()};
    for (const auto& pair : leakages) {
        const Leakage& leakage = pair.first;
        ofs << pair.second << " " << Edge::kind_tostr(leakage.com_kind) << " " << leakage.com << " " << leakage.comx << " " << leakage.desc << "\n";
    }
    
    ProfilerStop();
    
    return nleaks;
}

template <typename OutputIt>
OutputIt AEG::process_leakage(OutputIt out, const z3::eval& eval) {
#if 0
    // get xsaccess order
    const auto xsaccess_less = [&] (NodeRef a, NodeRef b) -> bool {
        return (bool) eval(Node::xsaccess_order_less(*this)(a, b));
    };
    
    std::set<NodeRef, decltype(xsaccess_less)> xsaccesses {xsaccess_less};
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (eval(node.xsaccess() && node.exec())) {
            xsaccesses.insert(ref);
        }
    }
#endif

#if 0
    NodeRefVec accesses;
    get_path(eval, std::back_inserter(accesses));
#endif
    
    // TODO: do this analysis in axiomatic land, just on rf, rfx, etc. relations?
    
    
    fol::Context<bool, fol::ConEval> fol_ctx(fol::Logic<bool>(), fol::ConEval(eval), *this);
    const auto reads = fol_ctx.node_rel_if([&] (NodeRef, const Node& node) {
        return node.is_read() && node.arch;
    });
    const auto writes = fol_ctx.node_rel_if([&] (NodeRef, const Node& node) {
        return node.is_write() && node.arch;
    });
    const auto accesses = reads + writes;
    const auto xswrites = fol_ctx.node_rel_if([] (NodeRef, const Node& node) { return node.exec() && node.xswrite; });
    const auto xsreads = fol_ctx.node_rel_if([] (NodeRef, const Node& node) { return node.exec() && node.xsread; });
    const auto entry = fol_ctx.node_rel(Inst::ENTRY, ExecMode::ARCH);
    const auto exit = fol_ctx.node_rel(Inst::EXIT, ExecMode::ARCH);
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
    const auto flags = fol::element<1>(addr);
    
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
            if (lookup(ref2).inst.kind == Inst::EXIT) {
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
                    return z3::implies(node.is_write() && node.arch, node.xswrite);
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
                        .desc = std::string("co without ") + Edge::kind_tostr(kind),
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
