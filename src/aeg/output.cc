#include "aeg.h"
#include "fol.h"

void AEG::dump_graph(const std::string& path) const {
    std::ofstream ofs {path};
    dump_graph(ofs);
}

void AEG::dump_graph(std::ostream& os) const {
    os << R"=(
    digraph G {
    overlap = scale;
    splines = true;
    
    )=";
    
    // define nodes
    unsigned next_id = 0;
    std::unordered_map<NodeRef, std::string> names;
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        const std::string name = std::string("n") + std::to_string(next_id);
        names.emplace(ref, name);
        ++next_id;
        
        os << name << " ";
        
        std::stringstream ss;
        ss << ref << " ";
        ss << *node.inst << "\n";
        ss << "po: " << node.arch << "\n"
        << "tfo: " << node.trans << "\n"
        << "tfo_depth: " << node.trans_depth << "\n";
        
        if (node.addr_def) {
            ss << "addr (def): " << *node.addr_def << "\n";
        }
        if (!node.addr_refs.empty()) {
            ss << "addr (refs):";
            for (const auto& ref : node.addr_refs) {
                ss << " " << ref.second;
            }
            ss << "\n";
        }
        
        if (dump_constraints) {
            ss << "constraints: " << node.constraints << "\n";
        }
        
        dot::emit_kvs(os, "label", ss.str());
        os << ";\n";
    }
    
    // define edges
    graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
        os << names.at(src) << " -> " << names.at(dst) << " ";
        dot::emit_kvs(os, "label", util::to_string(edge));
        os << ";\n";
    });
    
    // graph labels (constraints)
    {
        os << "graph ";
        std::stringstream ss;
        ss << constraints;
        dot::emit_kvs(os, "label", ss.str());
        os << "\n";
    }
    
    os << "}\n";
}

void AEG::output_execution(std::ostream& os, const z3::eval& eval, const EdgeSet& flag_edges) {
    os << R"=(
    digraph G {
    overlap = scale;
    splines = true;
    
    )=";
    
    // define nodes
    unsigned next_id = 0;
    std::unordered_map<NodeRef, std::string> names;
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (eval(node.exec())) {
            const std::string name = std::string("n") + std::to_string(next_id);
            names.emplace(ref, name);
            ++next_id;
            
            os << name << " ";
            std::stringstream ss;
            ss << ref << " " << *node.inst << "\n";

            if (node.inst->is_memory()) {
                ss << "{" << eval(node.get_memory_address()) << "} ";
            }
            
            const bool xsread = (bool) eval(node.xsread);
            const bool xswrite = (bool) eval(node.xswrite);
            if (xsread) {
                ss << "R";
            }
            if (xswrite) {
                ss << "W";
            }
            if ((xsread || xswrite) && node.xsaccess_order) {
                ss << "(" << eval(*node.xsaccess_order) << ") ";
            }
            
#if USE_TAINT
            // DEBUG: taint
            ss << " taint(" << eval(node.taint) << ")";
            if (node.inst->is_memory()) {
                const z3::expr flag = tainter->flag(ref);
                if (eval(flag)) {
                    ss << " FLAGGED";
                }
                
            }
            if (eval(node.taint_trans)) {
                ss << " taint_trans";
            }
#endif
            
            std::string color;
            if (eval(node.arch)) {
                color = "green";
            } else if (eval(node.trans)) {
                color = "red";
            }
            
            dot::emit_kvs(os, dot::kv_vec {{"label", ss.str()}, {"color", color}});
            os << ";\n";
        }
    }
    
    const auto output_edge = [&] (NodeRef src, NodeRef dst, Edge::Kind kind) {
        if (!include_edges.empty()) {
            if (include_edges.find(kind) == include_edges.end()) { return; }
        }
        
        os << names.at(src) << " -> " << names.at(dst) << " ";
        static const std::unordered_map<Edge::Kind, std::string> colors = {
            {Edge::TFO, "black"},
            {Edge::RF, "gray"},
            {Edge::CO, "blue"},
            {Edge::FR, "purple"},
            {Edge::RFX, "gray"},
            {Edge::COX, "blue"},
            {Edge::FRX, "purple"},
            {Edge::ADDR, "brown"},
            {Edge::CTRL, "purple"},
            {Edge::PO, "black"},
        };
        std::string color = colors.at(kind);
        if (flag_edges.find(std::make_tuple(src, dst, kind)) != flag_edges.end()) {
            color = "red";
        }
        dot::emit_kvs(os, dot::kv_vec {{"label", util::to_string(kind)}, {"color", color}});
        os << ";\n";
    };
    
    graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
        if (eval(edge.exists)) {
            output_edge(src, dst, edge.kind);
        }
    });
    
    const fol::Context<bool, fol::ConEval> fol_ctx {fol::Logic<bool>(), fol::ConEval(eval), *this};
    
    const auto output_rel = [&] (const auto& rel, Edge::Kind kind) {
        for (const auto& p : rel) {
            output_edge(std::get<0>(p.first), std::get<1>(p.first), kind);
        }
    };
    
    static const Edge::Kind rels[] = {Edge::RF, Edge::CO, Edge::FR, Edge::RFX, Edge::COX, Edge::FRX};
    for (const Edge::Kind kind : rels) {
        output_rel(fol_ctx.edge_rel(kind), kind);
    }
    
    // output pseudo com and comx
    using EdgeSig = std::tuple<NodeRef, NodeRef, Edge::Kind>;
    using EdgeSigVec = std::vector<EdgeSig>;
    EdgeSigVec com, comx;
        // get_concrete_com(model, std::back_inserter(com));
        // get_concrete_comx(model, std::back_inserter(comx));
    for (const auto& edge : com) {
        std::apply(output_edge, edge);
    }
    for (const auto& edge : comx) {
        std::apply(output_edge, edge);
    }
    
#if 1
    // add tfo rollback edges
    for (const NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (!eval(node.arch)) { continue; }
        const auto next = [&] (NodeRef ref, Edge::Kind kind) -> std::optional<NodeRef> {
            const auto tfos = get_nodes(Direction::OUT, ref, kind);
            const auto tfo_it = std::find_if(tfos.begin(), tfos.end(), [&] (const auto& x) -> bool {
                return (bool) eval(x.second);
            });
            if (tfo_it == tfos.end()) {
                return std::nullopt;
            } else {
                return tfo_it->first;
            }
        };
        std::optional<NodeRef> cur;
        NodeRef prev = ref;
        while ((cur = next(prev, Edge::TFO))) {
            prev = *cur;
        }
        const auto arch_dst = next(ref, Edge::PO);
        if (prev != ref && arch_dst) {
            const auto trans_src = prev;
            output_edge(trans_src, *arch_dst, Edge::TFO);
        }
    }
#endif
    
    os << "}\n";
}

void AEG::output_execution(const std::string& path, const z3::eval& eval, const EdgeSet& flag_edges) {
    std::ofstream ofs {path};
    output_execution(ofs, eval, flag_edges);
}
