#include "fol.h"
#include "aeg.h"

namespace fol {

binary_node_relation edge_rel_co(const AEG& aeg) {
    NodeRefVec writes;
    aeg.get_writes(std::back_inserter(writes));
    
    binary_node_relation rel;
    for (auto it1 = writes.begin(); it1 != writes.end(); ++it1) {
        for (auto it2 = writes.begin(); it2 != writes.end(); ++it2) {
            const auto f = aeg.co_exists(*it1, *it2);
            if (!f.is_false()) {
                const auto tuple = std::make_tuple(*it1, *it2);
                const AEG::Node& src = aeg.lookup(*it1);
                const AEG::Node& dst = aeg.lookup(*it2);
                rel.emplace(tuple, src.arch && dst.arch && src.same_addr(dst));
            }
        }
    }
    
    return rel;
}

binary_node_relation edge_rel(const AEG& aeg, UHBEdge::Kind kind) {
    switch (kind) {
        case AEG::Edge::Kind::CO:
            return edge_rel_co(aeg);
        default:
            break;
    }
    
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
    return node_rel(aeg, [&] (NodeRef, const AEG::Node& node) -> z3::expr {
        return aeg.ctx().bool_val(node.inst.kind == kind);
    });
}

}
