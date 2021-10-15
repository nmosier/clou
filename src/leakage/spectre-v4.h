#pragma once

#include "aeg/aeg.h"
#include "cfg/expanded.h"

template <typename OutputIt>
OutputIt AEG::leakage_spectre_v4(z3::solver& solver, OutputIt out) {
    const z3::scope scope {solver};
    
    // create mem array (used to find possible most-recent architectural writes)
    const MemsPair mems = {
        .arch = get_mems(&Node::arch),
        .trans = get_mems(&Node::trans),
    };
    
    for_each_edge(Edge::ADDR, [&] (const NodeRef addr_src, const NodeRef access3, const Edge& addr_edge) {
        const z3::scope scope {solver};
        const Node& access3_node = lookup(access3);

        // require access3 is trans
        solver.add(access3_node.trans, "access3.trans");
        
        // require access3 -rfx-> exit
        {
            z3::expr rfx = context.FALSE;
            for (NodeRef exit : exits) {
                const Node& exit_node = lookup(exit);
                rfx = rfx || (exit_node.arch && rfx_exists(access3, exit));
            }
            solver.add(rfx, "access3 -rfx-> exit");
        }

        leakage_spectre_v4_load2(solver, mems, addr_src, access3, out);
    });
    
    return out;
}

template <typename OutputIt>
void AEG::leakage_spectre_v4_load2(z3::solver& solver, const MemsPair& mems, NodeRef load2, NodeRef access3, OutputIt& out, unsigned traceback_depth) {
    std::cerr << __FUNCTION__ << ": load2=" << load2 << " access3=" << access3 << " depth=" << traceback_depth << "\n";
    
    if (traceback_depth > max_traceback) { return; }
    
    const z3::scope scope {solver};
    const Node& load2_node = lookup(load2);
    
    // require load2 trans
    solver.add(lookup(load2).trans, util::to_string("load2{", load2, ".trans").c_str());
    
    const auto get_store1_candidates = [&] (const Mems& mems, z3::expr& store1_sym) -> std::vector<z3::expr> {
        store1_sym = mems.at(load2)[load2_node.get_memory_address()];
        std::vector<z3::expr> store1_candidates;
        z3::enumerate(solver, store1_sym, std::back_inserter(store1_candidates));
        return store1_candidates;
    };
    
    // enumerate store1
    {
        z3::expr store1_arch_sym {context.context};
        const auto store1_arch_candidates = get_store1_candidates(mems.arch, store1_arch_sym);
        const z3::scope scope {solver};
        
        // ensure load is the first speculated instruction
        const auto tfo = get_nodes(Direction::IN, load2, Edge::TFO);
        for (const auto& pred : tfo) {
            solver.add(z3::implies(pred.second, lookup(pred.first).arch), util::to_string("pred-arch-", pred.first).c_str());
        }
        
        // recurse on all bindings for store1 candidates
        std::cerr << __FUNCTION__ << ": store1: " << store1_arch_candidates.size() << " candidates\n";
        for (const z3::expr& store1_arch_con : store1_arch_candidates) {
            const z3::scope scope {solver};
            const NodeRef store1 = store1_arch_con.get_numeral_uint64();
            solver.add(store1_arch_con == store1_arch_sym, "store1-arch"); // bind store1
            leakage_spectre_v4_store1(solver, mems, store1, load2, access3, out);
        }
    }
    
    // trace back
    {
        z3::expr store1_trans_sym {context.context};
        const auto store1_trans_candidates = get_store1_candidates(mems.trans, store1_trans_sym);
        for (const z3::expr& store1_trans_con : store1_trans_candidates) {
            const z3::scope scope {solver};
            const NodeRef store1 = store1_trans_con.get_numeral_uint64();
            solver.add(store1_trans_con == store1_trans_sym, util::to_string("store1-trans-", store1).c_str());
            for (Edge::Kind kind : std::array<Edge::Kind, 2> {Edge::ADDR, Edge::DATA}) {
                std::vector<std::pair<NodeRef, z3::expr>> nodes;
                get_nodes(Direction::IN, store1, std::back_inserter(nodes), kind);
                for (const auto& node : nodes) {
                    const z3::scope scope {solver};
                    solver.add(node.second, util::to_string("trace-back-", load2, "-", node.first).c_str());
                    leakage_spectre_v4_load2(solver, mems, node.first, access3, out, traceback_depth + 1);
                }
            }
        }
    }
}

template <typename OutputIt>
void AEG::leakage_spectre_v4_store1(z3::solver& solver, const MemsPair& mems, NodeRef store1, NodeRef load2, NodeRef access3, OutputIt& out) {
    std::cerr << __FUNCTION__ << ": store1=" << store1 << " load2=" << load2 << " access3=" << access3 << "\n";
    
    const z3::scope scope {solver};
    if (store1 == entry) { return; }
    const Node& store1_node = lookup(store1);
    
    // require store1.arch
    solver.add(store1_node.arch, "store1.arch");
    
    // require ! store1 -rfx-> load2
    solver.add(!rfx_exists(store1, load2), "!(store1 -rfx-> load2)");
    
    // recurse over bindings for store0
    {
        NodeRefVec order;
        po.reverse_postorder(std::back_inserter(order));
        const auto partial_order = po.make_partial_order();
        unsigned store0_bindings = 0;
        for (NodeRef store0 : order) {
            const z3::scope scope {solver};
            
            if (store0 == store1) { break; }
            const Node& node = lookup(store0);
            if (!(node.may_write() && partial_order(store0, store1))) { continue; }
            
            solver.add(node.same_addr(store1_node), "same_addr[store0, store1]");
            if (solver.check() != z3::sat) { continue; }
            
            ++store0_bindings;
            leakage_spectre_v4_store0(solver, mems, store0, store1, load2, access3, out);
        }
        
        std::cerr << __FUNCTION__ << ": store0: " << store0_bindings << " bindings\n";
    }
}

template <typename OutputIt>
void AEG::leakage_spectre_v4_store0(z3::solver& solver, const MemsPair& mem, NodeRef store0, NodeRef store1, NodeRef load2, NodeRef access3, OutputIt& out) {
    std::cerr << __FUNCTION__ << ": store0=" << store0 << " store1=" << store1 << " load2=" << load2 << " access3=" << access3 << "\n";
    
    const z3::scope scope {solver};
    
    solver.add(rfx_exists(store0, load2), "store0 -rfx-> load2");
    if (solver.check() != z3::sat) { return; }
    
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
    
    EdgeSet flag_edges;
    flag_edges.emplace(store0, load2, Edge::RFX);
    flag_edges.emplace(store0, store1, Edge::CO);
    flag_edges.emplace(access3, exit, Edge::RFX);
    std::stringstream ss;
    ss << output_dir << "/spectre-v4-s" << store0 << "-" << store1 << "-" << load2 << "-" << access3 << ".dot";
    std::cerr << "spectre-v4: saving to: " << ss.str() << "\n";
    output_execution(ss.str(), eval, flag_edges);
    
    *out++ = Leakage_SpectreV4 {
        .store0 =  store0,
        .store1 =  store1,
        .load2  =  load2,
        .access3 = access3,
    };
}
