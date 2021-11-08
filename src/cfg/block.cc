#include "block.h"
#include "cfg.h"
#include "util/algorithm.h"

void BlockCFG::construct(const CFG& cfg) {
    entry = cfg.entry;
    
    NodeRefVec todo = {cfg.entry};
    NodeRefSet done;
    while (!todo.empty()) {
        const NodeRef cur = todo.back();
        todo.pop_back();
        if (!done.insert(cur).second) {
            continue;
        }
        NodeRefVec block;
        construct_block(cfg, cur, block, std::back_inserter(todo));
        blocks.emplace(cur, std::move(block));
    }
}

template <typename OutputIt>
OutputIt BlockCFG::construct_block(const CFG& cfg, BlockID id, NodeRefVec& block, OutputIt out) {
    NodeRef cur = id;
    while (true) {
        block.push_back(cur);
        const auto& succs = cfg.po.fwd.at(cur);
        if (succs.empty()) {
            exits.insert(id);
            return out;
        } else if (succs.size() != 1) {
            return util::copy(succs, out);
        }
        const NodeRef next = *succs.begin();
        if (cfg.po.rev.at(next).size() != 1) {
            *out++ = next;
            return out;
        }
        cur = next;
    }
}
