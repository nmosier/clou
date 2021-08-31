#include "fol.h"
#include "aeg.h"

namespace fol {

binary_node_relation edge_rel(const AEG& aeg, UHBEdge::Kind kind) {
    binary_node_relation rel;
    aeg.for_each_edge(kind, [&rel] (const NodeRef src, const NodeRef dst, const UHBEdge& edge) {
        const auto tuple = std::make_tuple(src, dst);
        const auto it = rel.find(tuple);
        if (it == rel.end()) {
            rel.emplace(tuple, edge.exists);
        } else {
            it->second = it->second || edge.exists;
        }
    });
    return rel;
}

relation<NodeRef> node_rel(const AEG& aeg, Inst::Kind kind) {
    return node_rel(aeg, [&] (const auto& p) -> z3::expr {
        return aeg.ctx().bool_val(p.node.inst.kind == kind);
    });
}

}
