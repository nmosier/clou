#pragma once

#include <vector>
#include <unordered_map>

#include "lcm.h"
#include "aeg-po2.h"
#include "binrel.h"
#include "config.h"
#include "spec-prim.h" // for class Fork

class AEGPO_Unrolled;
struct SpeculationInfo;

class AEGPO_Expanded: public AEGPO {
public:
    explicit AEGPO_Expanded(unsigned num_specs): AEGPO(num_specs) {}
    
    void construct(const AEGPO_Unrolled& in, const SpeculationInfo& spec);
    
private:
    using NodeMap = std::unordered_map<NodeRef, NodeRef>;
    
    struct Task {
        NodeRef in_src;
        NodeRef in_dst;
        NodeRef src;
        unsigned spec_depth;
        std::vector<Fork> forks;
    };
    
    // DEBUG: expansion map
    std::unordered_map<NodeRef, NodeRefSet> expansions;
    
    template <typename OutputIt>
    void construct_rec(const AEGPO_Unrolled& in, const SpeculationInfo& spec, const Task& task, NodeMap& map, OutputIt out);
    
    void resolve_refs(const AEGPO_Unrolled& in);
};
