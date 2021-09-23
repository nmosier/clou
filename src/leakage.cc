#include <fstream>

#include "aeg.h"
#include "timer.h"
#include "fork_work_queue.h"
#include "llvm-util.h"

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


z3::expr AEG::leakage_rfx2() const {
    z3::expr acc = ctx().FALSE;
    
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        acc = acc || (addr_edge.exists && lookup(addr_dst).xswrite);
    });
    
    return acc;
}

z3::expr AEG::leakage_cox2() const {
    z3::expr acc = ctx().FALSE;
    
    // silent store check
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (node.is_write()) {
            acc = acc || !node.xswrite;
        }
    }
    
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        
        // check if there's a subsequent write/xswrite to the same address
        // TODO: test using a lambda function with this.
        for (NodeRef write : node_range()) {
            const Node& write_node = lookup(write);
            if (!write_node.is_write() || write_node.xswrite.is_false()) { continue; }
            acc = acc || (Node::xsaccess_order_less(*this)(addr_dst, write) && addr_dst_node.trans && write_node.xswrite && write_node.exec());
        }
        
    });
    
    return acc;
}

z3::expr AEG::leakage_frx2() const {
    
    /*
     frx leakage doesn't ever involve trans addr deps anyway. The only way frx leakage can arise is if A -fr-> B but B is not an xswrite, A is not an xsread, or B's xswrite happens before A's xsread.
     This can still be of interest when considering silent stores, etc.
     Consider a read A. It has to be an xsread. Moving along.
     Consider a write B. There will *always* be an incoming fr edge, with either an arch-pred instruction or program top. If it isn't an xswrite (e.g. is a silent store), the corresponding frx edge will be missing.
     Now, consider if the orders are reversed. Haven't covered this case yet.
     */
    
    z3::expr acc = ctx().FALSE;
    
    // NOTE: This is actually the same assertion as before, in leakage_cox2(). So omitting for now.
#if 0
    for (NodeRef write : node_range()) {
        const Node& write_node = lookup(write);
        acc = acc || !write_node.xswrite;
    }
#endif
    
    return acc;
}

unsigned AEG::leakage2(z3::solver& solver, unsigned max) const {
    solver.push();
    
    unsigned nleaks = 0;
    
    const z3::expr leakage = leakage_rfx2() || leakage_cox2() || leakage_frx2();
    solver.add(leakage, "leakage");
    
    for (unsigned i = 0; i < max; ++i) {
        const z3::check_result res = solver.check();
        std::cerr << res << "\n";
        switch (res) {
            case z3::unsat: goto done;
            case z3::sat: {
                const z3::model model = solver.get_model();
                
                // add constraints
                std::cerr << "adding different solution constraints...\n";
                Stopwatch timer;
                timer.start();
                std::vector<z3::expr> exprs;
                auto it = std::back_inserter(exprs);
                for (const Node& node : nodes) {
                    *it++ = node.arch;
                    *it++ = node.trans;
                }
                
                for_each_edge([&] (NodeRef, NodeRef, const Edge& edge) {
                    *it++ = edge.exists;
                });
                
                const z3::expr same_sol = std::transform_reduce(exprs.begin(), exprs.end(), context.TRUE, util::logical_and<z3::expr>(), [&] (const z3::expr& e) -> z3::expr {
                    return e == model.eval(e);
                });
                
                solver.add(!same_sol);
                
                // TODO: process concrete output to get all leakage
                
                ++nleaks;
                
                break;
            }
            case z3::unknown: throw std::logic_error("unknown");
        }
    }
    
done:
    solver.pop();
    return nleaks;
}



template <typename OutputIt>
void AEG::leakage_rfx(NodeRef read, z3::solver& solver, OutputIt out) const {
    const Node& read_node = lookup(read);
    assert(read_node.is_read());
    
    z3::expr addr = context.FALSE;
    std::vector<std::tuple<z3::expr, NodeRef>> causes;
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        if (addr_dst_node.xswrite.is_false()) { std::abort(); }
        const auto rfx = rfx_exists(addr_dst, read);
        const z3::expr f = addr_edge.exists && rfx && addr_dst_node.trans;
        addr = addr || f;
        *out++ = std::make_tuple(f, addr_dst, read);
    });
    
    solver.add(read_node.arch && addr, "leakage-rfx");
}

template <typename OutputIt>
void AEG::leakage_cox(NodeRef write, z3::solver& solver, OutputIt out) const {
    const Node& write_node = lookup(write);
    assert(write_node.is_write());
    
    z3::expr addr = context.FALSE;
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        if (addr_dst_node.xswrite.is_false()) { std::abort(); }
        const auto cox = cox_exists(addr_dst, write);
        const z3::expr f = addr_edge.exists && cox && addr_dst_node.trans;
        addr = addr || f;
        *out++ = std::make_tuple(f, addr_dst, write);
    });
    
    solver.add(write_node.arch && addr, "leakage-cox");
}

template <typename OutputIt>
void AEG::leakage_frx(NodeRef write, z3::solver& solver, OutputIt out) const {
    const Node& write_node = lookup(write);
    assert(write_node.is_write());
    
    z3::expr addr = context.FALSE;
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        if (addr_dst_node.xswrite.is_false()) { std::abort(); }
        const auto frx = frx_exists(addr_dst, write);
        const z3::expr f = addr_edge.exists && frx && addr_dst_node.trans;
        addr = addr || f;
        *out++ = std::make_tuple(f, addr_dst, write);
    });
    
    solver.add(write_node.arch && addr, "leakage-frx");
}

unsigned AEG::leakage(z3::solver& solver) {
    z3::model model {solver.ctx()};
    unsigned nleaks = 0;
    
    std::vector<std::tuple<z3::expr, NodeRef, NodeRef>> causes;
    const auto out = std::back_inserter(causes);
    
    const auto process = [&] (const std::string& type, NodeRef ref, z3::solver& solver) {
        z3::check_result res;
        {
            Timer timer;
            res = solver.check();
        }
        if (res == z3::sat) {
            std::stringstream ss;
            ss << "out/leakage-" << type << "-" << ref;
            const z3::model model = solver.get_model();
            output_execution(ss.str() + ".dot", model);
            
            // attribute leakage
            std::ofstream txt {ss.str() + ".txt"};
            for (const auto& cause : causes) {
                if (model.eval(std::get<0>(cause)).is_true()) {
                    const NodeRef src = std::get<1>(cause);
                    const NodeRef dst = std::get<2>(cause);
                    txt << type << " " << src << " " << dst << "\n";
                    const Node& src_node = lookup(src);
                    const Node& dst_node = lookup(dst);
                    txt << src << " " << src_node.inst << "\n";
                    txt << dst << " " << dst_node.inst << "\n";
                    txt << "\n";
                }
            }
            
        }
        std::cerr << type << " " << res << "\n";
    };
    
    fork_work_queue queue {num_jobs};
    
    for (NodeRef read : node_range()) {
        const Node& read_node = lookup(read);
        if (read_node.is_read()) {
            queue.push([&, read] {
                leakage_rfx(read, solver, out);
                process("rfx", read, solver);
                return 0;
            });
        }
    }
    
    for (NodeRef write : node_range()) {
        const Node& write_node = lookup(write);
        if (write_node.is_write()) {
            queue.push([&, write] {
                leakage_cox(write, solver, out);
                process("cox", write, solver);
                return 0;
            });
        }
    }
    
    for (NodeRef write : node_range()) {
        const Node& write_node = lookup(write);
        if (write_node.is_write()) {
            queue.push([&, write] {
                leakage_frx(write, solver, out);
                process("frx", write, solver);
                return 0;
            });
        }
    }
    
    std::vector<std::pair<int, std::size_t>> results;
    queue.run(std::back_inserter(results));
    // TODO: use results somehow
    
    return nleaks;
}
