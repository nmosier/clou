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
    
    for (const auto& p : blocks) {
        for (NodeRef ref : p.second) {
            ref2block.emplace(ref, p.first);
        }
    }
    
    construct_rel(cfg);
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

void BlockCFG::construct_rel(const CFG& cfg) {
    for (const auto& block_pair : blocks) {
        const auto& block = block_pair.second;
        const auto first = block.front();
        const auto last = block.back();
        po.add_node(first);
        
        const auto f = [&] (const NodeRefSet& in, NodeRefSet& out) {
            std::transform(in.begin(), in.end(), std::inserter(out, out.end()), [&] (NodeRef in) -> NodeRef {
                return ref2block.at(in);
            });
        };
        f(cfg.po.fwd.at(last), po.fwd[first]);
        f(cfg.po.rev.at(first), po.rev[first]);
    }
}

CFGOrder::CFGOrder(const CFG& cfg): bcfg(cfg) {
    NodeRefVec order;
    bcfg.po.reverse_postorder(std::back_inserter(order), bcfg.entry);
    
    NodeRefMap ins;
    NodeRefMap outs;
    for (NodeRef ref : order) {
        NodeRefSet set;
        for (NodeRef pred : bcfg.po.rev.at(ref)) {
            util::copy(outs.at(pred), std::inserter(set, set.end()));
        }
        ins.emplace(ref, set);
        set.insert(ref);
        outs.emplace(ref, set);
    }
    map = std::move(ins);
}

bool CFGOrder::operator()(NodeRef a, NodeRef b) const {
    // convert to block
    const auto a_blk = bcfg.ref2block.at(a);
    const auto b_blk = bcfg.ref2block.at(b);
    if (a_blk == b_blk) {
        const auto& block = bcfg.blocks.at(a_blk);
        const auto a_it = std::find(block.begin(), block.end(), a);
        const auto b_it = std::find(block.begin(), block.end(), b);
        return a_it < b_it;
    } else {
        const auto& b_preds = map.at(b_blk);
        return b_preds.find(a_blk) != b_preds.end();
    }
}
