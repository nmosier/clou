#pragma once

#include "aeg/aeg.h"
#include "cfg/expanded.h"

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
    
    
    if (loads.size() == 2) {
        assert(solver.check() == z3::sat);
        const z3::eval eval {solver.get_model()};
        
        *out++ = Leakage_SpectreV1_Classic {
            .load0 = loads.at(1),
            .load1 = loads.at(0),
            .transmitter2 = transmitter,
        };
        
        // find exit
        const NodeRef exit = exit_con(eval);
        const auto rfx_edge = util::push(flag_edges, {transmitter, exit, Edge::RFX});
        
        std::cerr << "FLAG EDGES: " << flag_edges << "\n";
        
        const std::string path = leakage_get_path("spectre-v1", {loads.at(1), loads.at(0), transmitter});
        output_execution(path, eval, flag_edges);
    }
    
    if (loads.size() >= 2) {
        return;
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
            
            const auto addr_edge = util::push(flag_edges, std::make_tuple(load, access, Edge::ADDR));
            
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
OutputIt AEG::leakage_spectre_v1_control(z3::solver& solver, OutputIt out) {
    const z3::scope scope {solver};
    
    const auto mems = get_exec_mems();
    
    for_each_edge(Edge::CTRL, [&] (const NodeRef ctrl_src, const NodeRef ctrl_dst, const Edge& ctrl_edge) {
        const z3::scope scope {solver};
        
        const NodeRef transmitter = ctrl_dst;
        const Node& transmitter_node = lookup(transmitter);
        const NodeRef access = ctrl_dst;
        
        // require ctrl_edge.exists
        solver.add(ctrl_edge.exists, "ctrl_edge.exists");
        
        // require transmitter.trans
        solver.add(transmitter_node.trans, "transmitter.trans");
        
        // require transmitter -rfx-> exit
        {
            const z3::expr f = std::transform_reduce(exits.begin(), exits.end(), context.FALSE, util::logical_or<z3::expr>(),
                                                     [&] (const NodeRef exit) -> z3::expr {
                return lookup(exit).arch && rfx_exists(transmitter, exit);
            });
            solver.add(f, "transmitter -rfx-> exit");
        }
        
        // recurse
        EdgeVec flag_edges;
        leakage_spectre_v1_control_rec(solver, out, mems, transmitter, flag_edges, 0);
    });
}

template <typename OutputIt>
void AEG::leakage_spectre_v1_control_rec(z3::solver& solver, OutputIt& out, const Mems& mems, NodeRef transmitter, EdgeVec& flag_edges, unsigned traceback_depth) {
    const z3::scope scope {solver};
    
    
}
