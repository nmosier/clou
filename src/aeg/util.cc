#include "aeg.h"
#include "cfg/expanded.h"

namespace aeg {

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

}
