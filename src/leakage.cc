#include "aeg.h"
#include "timer.h"
#include "fork_work_queue.h"

void AEG::leakage_rfx(NodeRef read, z3::solver& solver) const {
    const Node& read_node = lookup(read);
    assert(read_node.is_read());

#if 0
    // get xswrite_order of corresponding write.
    const z3::expr& xsread_order = read_node.xsread_order;
    z3::expr xswrite_order = read_node.mem[read_node.get_memory_address()];
#endif
    
    z3::expr addr = context.FALSE;
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        if (addr_dst_node.xswrite.is_false()) { std::abort(); }
        const auto rfx = rfx_exists(addr_dst, read);
        addr = addr || (addr_edge.exists && rfx && addr_dst_node.trans);
    });
    
    solver.add(read_node.arch && addr, "leakage-rfx");
}

void AEG::leakage_cox(NodeRef write, z3::solver& solver) const {
    const Node& write_node = lookup(write);
    assert(write_node.is_write());
    
    z3::expr addr = context.FALSE;
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        if (addr_dst_node.xswrite.is_false()) { std::abort(); }
        const auto cox = cox_exists(addr_dst, write);
        addr = addr || (addr_edge.exists && cox && addr_dst_node.trans);
    });
    
    solver.add(write_node.arch && addr, "leakage-cox");
}

void AEG::leakage_frx(NodeRef write, z3::solver& solver) const {
    const Node& write_node = lookup(write);
    assert(write_node.is_write());
    
    z3::expr addr = context.FALSE;
    for_each_edge(Edge::ADDR, [&] (NodeRef addr_src, NodeRef addr_dst, const Edge& addr_edge) {
        const Node& addr_dst_node = lookup(addr_dst);
        if (addr_dst_node.xswrite.is_false()) { std::abort(); }
        const auto frx = frx_exists(addr_dst, write);
        addr = addr || (addr_edge.exists && frx && addr_dst_node.trans);
    });
    
    solver.add(write_node.arch && addr, "leakage-frx");
}

unsigned AEG::leakage(z3::solver& solver) const {
    z3::model model {solver.ctx()};
    unsigned nleaks = 0;
    
    const auto process = [&] (const std::string& type, NodeRef ref, z3::solver& solver) {
        z3::check_result res;
        {
            Timer timer;
            res = solver.check();
        }
        if (res == z3::sat) {
            std::stringstream ss;
            ss << "out/leakage-" << type << "-" << ref << ".dot";
            output_execution(ss.str(), solver.get_model());
            ++nleaks;
        }
        std::cerr << type << " " << res << "\n";
    };
    
    fork_work_queue queue {num_jobs};
    
    for (NodeRef read : node_range()) {
        const Node& read_node = lookup(read);
        if (read_node.is_read()) {
            queue.push([&, read] {
                leakage_rfx(read, solver);
                process("rfx", read, solver);
            });
        }
    }
    
    for (NodeRef write : node_range()) {
        const Node& write_node = lookup(write);
        if (write_node.is_write()) {
            queue.push([&, write] {
                leakage_cox(write, solver);
                process("cox", write, solver);
            });
        }
    }
    
    for (NodeRef write : node_range()) {
        const Node& write_node = lookup(write);
        if (write_node.is_write()) {
            queue.push([&, write] {
                leakage_frx(write, solver);
                process("frx", write, solver);
            });
        }
    }
    
    std::vector<int> results;
    queue.run(std::back_inserter(results));
    // TODO: use results somehow
    
    return nleaks;
}
