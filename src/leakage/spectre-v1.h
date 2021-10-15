#pragma once

#include "aeg/aeg.h"
#include "cfg/expanded.h"

template <typename OutputIt>
OutputIt AEG::leakage_spectre_v1(z3::solver& solver, OutputIt out) {
    const z3::scope scope {solver};

    const auto mems = get_exec_mems();
    
    for_each_edge(Edge::ADDR, [&] (const NodeRef addr_src, const NodeRef addr_dst, const Edge& addr_edge) {
        const z3::scope scope {solver};
        
        const NodeRef secret0 = addr_src;
        const NodeRef transmitter1 = addr_dst;
        const Node& transmitter1_node = lookup(transmitter1);
        
        // require addr_edge.exists
        solver.add(addr_edge.exists, "addr_edge.exists");
        
        // require transmitter1.trans
        solver.add(transmitter1_node.trans, "transmitter1.trans");
        
        // require transmitter1.xswrite
        // NOTE: this is actually redundant with the rfx check.
        
        // require transmitter1 -rfx-> exit
        {
            const z3::expr f = std::transform_reduce(exits.begin(), exits.end(), context.FALSE, util::logical_or<z3::expr>(), [&] (const NodeRef exit) -> z3::expr {
                return lookup(exit).arch && rfx_exists(transmitter1, exit);
            });
            solver.add(f, "transmitter1 -rfx-> exit");
        }
        
        // recurse on secret0 candidates
        EdgeSet flag_edges = {{secret0, transmitter1, Edge::ADDR}};
        leakage_spectre_v1_secret0(solver, mems, secret0, transmitter1, out, 0, flag_edges);
    });
    
    return out;
}


template <typename OutputIt>
void AEG::leakage_spectre_v1_secret0(z3::solver& solver, const Mems& mems, NodeRef secret0, NodeRef transmitter1, OutputIt& out, unsigned traceback_depth, EdgeSet flag_edges) {
    std::cerr << __FUNCTION__ << ": secret0=" << secret0 << " transmitter1=" << transmitter1 << " depth=" << traceback_depth << "\n";
    
    if (traceback_depth > max_traceback) { return; }

    const z3::scope scope {solver};
    const Node& secret0_node = lookup(secret0);
    
    // require secret0.exec
    solver.add(secret0_node.exec(), "secret0.exec");

    // find leakage
    {
        if (solver.check() == z3::sat) {
            const z3::eval eval {solver.get_model()};
            
            // find exit
            const auto get_exit = [&] () -> NodeRef {
                for (const NodeRef exit : exits) {
                    if (eval(lookup(exit).arch)) {
                        return exit;
                    }
                }
                std::abort(); // no exit found! logic error
            };
            const NodeRef exit = get_exit();

            EdgeSet flag_edges2 = flag_edges;
            flag_edges2.emplace(transmitter1, exit, Edge::RFX);
            std::stringstream ss;
            ss << output_dir << "/spectre-v1-" << secret0 << "-" << transmitter1 << ".dot";
            std::cerr << "spectre-v1: saving to: " << ss.str() << "\n";
            output_execution(ss.str(), eval, flag_edges2);
            
            *out++ = Leakage_SpectreV1_Classic {
                .secret0 = secret0,
                .transmitter1 = transmitter1,
            };
        }
    }
    
    // traceback
    {
        std::vector<z3::expr> tracebacks;
        const z3::expr traceback_sym = mems.at(secret0)[lookup(secret0).get_memory_address()];
        z3::enumerate(solver, traceback_sym, std::back_inserter(tracebacks));
        assert(!tracebacks.empty());
        
        std::cerr << __FUNCTION__ << ": tracebacks: " << tracebacks << "\n";
        
        // TODO: extract traceback function, since it's shared between Spectre v1 and v4 drivers.
        for (const z3::expr& traceback_con : tracebacks) {
            const z3::scope scope {solver};
            const NodeRef traceback = traceback_con.get_numeral_uint64();
            solver.add(traceback_con == traceback_sym, util::to_string("traceback-rf-", traceback).c_str());
            for (Edge::Kind kind : std::array<Edge::Kind, 2> {Edge::ADDR, Edge::DATA}) {
                const auto nodes = get_nodes(Direction::IN, traceback, kind);
                for (const auto& node : nodes) {
                    const z3::scope scope {solver};
                    solver.add(node.second, util::to_string("traceback-addr/data-", node.first).c_str());
                    leakage_spectre_v1_secret0(solver, mems, node.first, transmitter1, out, traceback_depth + 1, flag_edges);
                }
            }
        }
    }
}
