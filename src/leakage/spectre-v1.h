#pragma once

#include "aeg/aeg.h"
#include "cfg/expanded.h"

/* Make this more general.
 * Need to find _n_ loads, traced back via addr dependencies connected via any number of rf * (addr + data) edges
 *
 */

#if 0
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
        leakage_spectre_v1_load(solver, mems, secret0, transmitter1, out, 0, flag_edges);
    });
    
    return out;
}


template <typename OutputIt>
void AEG::leakage_spectre_v1_load(z3::solver& solver, const Mems& mems, NodeRef secret0, NodeRef transmitter1, OutputIt& out, unsigned traceback_depth, EdgeSet flag_edges) {
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
#endif

/*
 * Alternative approach. Should be simpler.
 *
 * Init: load -ADDR-> access.
 *
 */

template <typename OutputIt>
OutputIt AEG::leakage_spectre_v1(z3::solver& solver, OutputIt out) {
    const z3::scope scope {solver};
    
    const auto mems = get_exec_mems();
    
    for_each_edge(Edge::ADDR, [&] (const NodeRef addr_src, const NodeRef addr_dst, const Edge& addr_edge) {
        const z3::scope scope {solver};
        
        const NodeRef transmitter = addr_dst;
        const Node& transmitter_node = lookup(transmitter);
        const NodeRef access = addr_dst;
        
        
        // require addr_edge.exists
        solver.add(addr_edge.exists, "addr_edge.exists");
        
        // require transmitter1.trans
        solver.add(transmitter_node.trans, "transmitter1.trans");
        
        // require transmitter1.xswrite
        // NOTE: this is actually redundant with the rfx check.
        
        // require transmitter1 -rfx-> exit
        {
            const z3::expr f = std::transform_reduce(exits.begin(), exits.end(), context.FALSE, util::logical_or<z3::expr>(),
                                                     [&] (const NodeRef exit) -> z3::expr {
                return lookup(exit).arch && rfx_exists(transmitter, exit);
            });
            solver.add(f, "transmitter -rfx-> exit");
        }
        
        // recurse on secret0 candidates
        EdgeVec flag_edges;
        NodeRefVec loads;
        leakage_spectre_v1_rec(solver, mems, loads, transmitter, out, flag_edges, access, 0);
    });
    
    return out;
}

template <typename OutputIt>
void AEG::leakage_spectre_v1_rec(z3::solver& solver, const Mems& mems, std::vector<NodeRef>& loads, NodeRef transmitter, OutputIt& out, EdgeVec& flag_edges, NodeRef access, unsigned traceback_depth) {
    std::stringstream ss;
    ss << "loads=" << loads << " transmitter=" << transmitter << " access=" << access;
    const std::string desc = ss.str();
    
    std::cerr << desc << ": entry" << ": loads=" << loads << " transmitter=" << transmitter << " access=" << access << " depth=" << traceback_depth << "\n";
    
    if (loads.size() > 2) {
        return;
    }
    
    if (loads.size() == 2) {
        *out++ = Leakage_SpectreV1_Classic {
            .load0 = loads.at(1),
            .load1 = loads.at(0),
            transmitter,
        };
    }
    
    const z3::scope scope {solver};

    /* Trace back possible loads along ADDR edges */
    {
        const auto addrs = get_nodes(Direction::IN, access, Edge::ADDR);
        for (const auto& addr : addrs) {
            const z3::scope scope {solver};
            const NodeRef load = addr.first;
            const auto push_load = util::push(loads, load);
            solver.add(addr.second, util::to_string("addr-", access, "-", load).c_str());
            
            std::cerr << desc << ": commit" << ": committing load " << load << "\n";
            
            leakage_spectre_v1_rec(solver, mems, loads, transmitter, out, flag_edges, load, 0);
        }
    }
    
    /* Trace back load to store via RF */
    const Node& access_node = lookup(access);
    if (access_node.may_read()) {
        const z3::scope scope {solver};
        solver.add(access_node.read, util::to_string("access", access, ".read").c_str());

        const z3::expr store_sym = mems.at(access)[access_node.get_memory_address()];
        std::vector<z3::expr> stores;
        z3::enumerate(solver, store_sym, std::back_inserter(stores));
        
        for (const z3::expr& store_con : stores) {
            const NodeRef store = store_con.get_numeral_uint();
            const z3::scope scope {solver};
            solver.add(store_con == store_sym, util::to_string("store=", store).c_str());
            const auto rf_edge = util::push(flag_edges, std::make_tuple(store, access, Edge::RF));
            
            std::cerr << desc << ": traceback rf: " << access << " to " << store << "\n";
            
            leakage_spectre_v1_rec(solver, mems, loads, transmitter, out, flag_edges, store, traceback_depth + 1);
        }
    }
    
    
    /* Trace back store to load via ADDR, DATA */
    {
        std::vector<std::pair<NodeRef, z3::expr>> edges;
        get_nodes(Direction::IN, access, std::back_inserter(edges), Edge::ADDR);
        get_nodes(Direction::IN, access, std::back_inserter(edges), Edge::DATA);
        
        
        if (get_nodes(Direction::IN, access, Edge::DATA).empty()) {
            std::cerr << "empty DATA\n";
        }
        
        
        for (const auto& edge : edges) {
            const z3::scope scope {solver};
            solver.add(edge.second);
            
            std::cerr << desc << ": traceback" << ": tracing back store " << access << " to load " << edge.first << "\n";
            
            const auto addr_edge = util::push(flag_edges, std::make_tuple(edge.first, access, Edge::ADDR));
            const auto data_edge = util::push(flag_edges, std::make_tuple(edge.first, access, Edge::DATA));
            
            leakage_spectre_v1_rec(solver, mems, loads, transmitter, out, flag_edges, edge.first, traceback_depth + 1);
        }
    }
    
}


template <typename OutputIt>
void AEG::leakage_spectre_v1_traceback(z3::solver& solver, const Mems& mems, const std::vector<NodeRef>& loads, OutputIt& out, EdgeSet flag_edges, NodeRef store, unsigned traceback_depth) {
    
    const z3::scope scope {solver};
    
    
}
