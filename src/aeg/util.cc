#include "aeg.h"
#include "cfg/expanded.h"
#include "util/algorithm.h"

namespace aeg {

// TODO: delete
NodeRefSet AEG::spectrev4_siblings(NodeRef ref) const {
    NodeRefSet set;
    const Node& node = lookup(ref);
    for (const NodeRef parent : po.po.rev.at(ref)) {
        for (const NodeRef sib : po.po.fwd.at(parent)) {
            const Node& sib_node = lookup(sib);
            if (node.inst->get_inst() == sib_node.inst->get_inst()) {
                set.insert(sib);
            }
        }
    }
    return set;
}

void AEG::for_each_pred_in_window(NodeRef ref, unsigned window, std::function<void (NodeRef)> is, std::function<void (NodeRef)> isnt) {
    NodeRefVec order;
    po.postorder(std::back_inserter(order));
    
    constexpr unsigned max = std::numeric_limits<unsigned>::max();
    std::unordered_map<NodeRef, unsigned> map = {{ref, 0}};
    for (NodeRef ref : order) {
        auto it = map.find(ref);
        if (it != map.end()) { continue; }
        unsigned& acc = map[ref];
        acc = util::transform_min<unsigned>(po.po.fwd.at(ref), [&map] (NodeRef ref) -> unsigned {
            auto it = map.find(ref);
            if (it == map.end()) {
                return max;
            } else {
                return it->second;
            }
        });
        if (acc < max) {
            ++acc;
        }
    }
    
    for (NodeRef ref : order) {
        const auto it = map.find(ref);
        if (ref == entry ||
            exits.find(ref) != exits.end() ||
            (it != map.end() && it->second <= window)) {
            is(ref);
        } else {
            isnt(ref);
        }
    }
}

}
