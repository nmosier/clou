#pragma once

#include <vector>
#include <unordered_map>
#include <map>
#include <optional>

#include "lcm.h"
#include "cfg/cfg.h"
#include "binrel.h"
#include "config.h"

class CFG_Unrolled;
class CFG_Calls;
struct SpeculationInfo;

class CFG_Expanded: public CFG {
public:
    struct Exec {
        Option arch, trans;
        
        bool operator==(const Exec& other) const {
            return arch == other.arch && trans == other.trans;
        }
    };
    std::unordered_map<NodeRef, Exec> execs;
    
    explicit CFG_Expanded(unsigned num_specs): CFG(num_specs) {}
    
    void construct(const CFG& in);
    
private:
    using NodeMap = std::unordered_map<NodeRef, NodeRef>;
    
    template <typename Fork>
    struct Task {
        NodeRef in_dst;
        NodeRef src;
        Fork fork;
    };
    
    // DEBUG: expansion map
    std::unordered_map<NodeRef, NodeRefSet> expansions;
    
    template <typename Expand>
    void construct_full(const CFG& in, Expand& expand);
    
    void construct_partial(const CFG& in);
    using RefMap1 = std::unordered_map<Translations::Key, NodeRefSet, Translations::Key::Hash>;
    
    void resolve_single_ref(const llvm::Instruction *I, const llvm::Value *V, const CFG &in, std::vector<std::optional<RefMap1>>& maps, CFG::Node &node, NodeRef ref);
    
    void resolve_refs(const CFG& in);
};

