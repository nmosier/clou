#pragma once

#include <unordered_map>

#include "graph.h"
#include "noderef.h"

class CFG;

class BlockCFG {
public:
    using BlockID = NodeRef;
    using Rel = binrel<BlockID>;
    Rel po;
    BlockID entry;
    NodeRefSet exits;
    std::unordered_map<BlockID, NodeRefVec> blocks;
    
    BlockCFG(const CFG& cfg) {
        construct(cfg);
    }
    
private:
    
    void construct(const CFG& cfg);
    
    template <typename OutputIt>
    OutputIt construct_block(const CFG& cfg, const BlockID id, NodeRefVec& block, OutputIt out);
};
