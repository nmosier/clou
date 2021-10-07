#include "cfg/calls.h"
#include "cfg/unrolled.h"

void CFG_Calls::construct(const AEGPO& in) {
    std::unordered_map<NodeRef, NodeRefSet> map;
    
    // copy nodes
    for (NodeRef in_ref = 0; in_ref < in.size(); ++in_ref) {
        const Node& in_node = in.lookup(in_ref);
        NodeRefSet& set = map[in_ref];
        if (const auto *I = std::get_if<const llvm::Instruction *>(&in_node.v)) {
            if (const auto *C = llvm::dyn_cast<llvm::CallBase>(*I)) {
                for (const llvm::Value *arg : C->args()) {
                    if (arg->getType()->isPointerTy()) {
                        const NodeRef ref = add_node(Node(AEGPO::Node::Call {
                            .C = C,
                            .arg = arg
                        }, in_node.id));
                        set.insert(ref);
                    }
                }
            }
        }
        if (set.empty()) {
            const NodeRef ref = add_node(in_node);
            set.insert(ref);
        }
    }
    
    // add connections
    for (NodeRef in_src = 0; in_src < in.size(); ++in_src) {
        const NodeRefSet& srcs = map.at(in_src);
        for (NodeRef in_dst : in.po.fwd.at(in_src)) {
            const NodeRefSet& dsts = map.at(in_dst);
            for (NodeRef src : srcs) {
                for (NodeRef dst : dsts) {
                    add_edge(src, dst);
                }
            }
        }
    }
    
    entry = 0;
    
    // convert translations
    translations = in.translations;
}
