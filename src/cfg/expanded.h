#pragma once

#include <vector>
#include <unordered_map>

#include "lcm.h"
#include "cfg/cfg.h"
#include "binrel.h"
#include "config.h"
#include "spec-prim.h" // for class Fork

class AEGPO_Unrolled;
class CFG_Calls;
struct SpeculationInfo;

class AEGPO_Expanded: public AEGPO {
public:
    explicit AEGPO_Expanded(unsigned num_specs): AEGPO(num_specs) {}
    
    void construct(const AEGPO& in, const SpeculationInfo& spec);
    
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
    void construct_rec(const AEGPO& in, const SpeculationInfo& spec, const Task& task, NodeMap& map, OutputIt out);
    
    using RefMap = std::unordered_map<Translations::Key, NodeRefSet, Translations::Key::Hash>;
    void resolve_single_ref(const llvm::Instruction *I, const llvm::Value *V, const AEGPO &in, std::unordered_map<NodeRef, RefMap> &maps, AEGPO::Node &node, NodeRef ref);
    
    void resolve_refs(const AEGPO& in);
};
