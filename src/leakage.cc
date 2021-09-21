#include <fstream>

#include "aeg.h"
#include "timer.h"
#include "fork_work_queue.h"
#include "llvm-util.h"

template <typename OutputIt>
void AEG::leakage_rfx(NodeRef read, z3::solver& solver, OutputIt out) const {
    const Node& read_node = lookup(read);
    assert(read_node.is_read());

#if 0
    // get xswrite_order of corresponding write.
    const z3::expr& xsread_order = read_node.xsread_order;
    z3::expr xswrite_order = read_node.mem[read_node.get_memory_address()];
#endif
    
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

unsigned AEG::leakage(z3::solver& solver) const {
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
