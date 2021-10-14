#pragma once

#include <vector>
#include <unordered_map>

#include "lcm.h"
#include "cfg/cfg.h"
#include "binrel.h"
#include "config.h"
#include "spec-prim.h" // for class Fork

class CFG_Unrolled;
class CFG_Calls;
struct SpeculationInfo;

class CFG_Expanded: public CFG {
public:
    explicit CFG_Expanded(unsigned num_specs): CFG(num_specs) {}
    
    // TODO: erase
    void construct2(const CFG& in, const SpeculationInfo& spec);
    
    template <typename Expand>
    void construct(const CFG& in, Expand& expand);
    
    void construct_spectre_v1(const CFG& in);
    void construct_spectre_v4(const CFG& in);
    
private:
    using NodeMap = std::unordered_map<NodeRef, NodeRef>;
    
    struct Task {
        NodeRef in_src;
        NodeRef in_dst;
        NodeRef src;
        unsigned spec_depth;
        std::vector<Fork> forks;
    };
    
    template <typename Fork>
    struct Task2 {
        NodeRef in_dst;
        NodeRef src;
        Fork fork;
    };
    
    // DEBUG: expansion map
    std::unordered_map<NodeRef, NodeRefSet> expansions;
    
    // TODO: remove
    template <typename OutputIt>
    void construct_rec(const CFG& in, const SpeculationInfo& spec, const Task& task, NodeMap& map, OutputIt out);
    
    
    using RefMap = std::unordered_map<Translations::Key, NodeRefSet, Translations::Key::Hash>;
    void resolve_single_ref(const llvm::Instruction *I, const llvm::Value *V, const CFG &in, std::unordered_map<NodeRef, RefMap> &maps, CFG::Node &node, NodeRef ref);
    
    void resolve_refs(const CFG& in);
};


// TODO: move to its own class?
template <typename Fork>
struct Expand {
    using fork_type = Fork;
    
    template <typename OutputIt>
    OutputIt transition(const Fork& fork, NodeRef ref, OutputIt out);
    
    std::optional<NodeRef> merge(const Fork& fork, NodeRef ref);
};

struct Expand_Basic {
    const CFG& cfg;
    unsigned num_specs;
    
    
    Expand_Basic(const CFG& cfg, unsigned num_specs): cfg(cfg), num_specs(num_specs) {}
    
    struct Fork {
        unsigned spec_depth;
    };
    
    /** Transition function. Given a current fork and in noderef, output list of successor forks to process.
     */
    template <typename OutputIt>
    OutputIt transition(const Fork& fork_src, NodeRef in_src, NodeRef in_dst, OutputIt out) const {
        const unsigned new_spec_depth = std::min(fork_src.spec_depth, num_specs) + 1;
        *out++ = Fork {new_spec_depth};
        return out;
    }
    
    Fork init() const {
        return Fork {.spec_depth = num_specs + 1};
    }
    
    // virtual NodeRef merge(const Fork& fork, NodeRef in_ref, NodeRef out_ref) = 0;
};

struct Expand_SpectreV1: Expand_Basic {
    using Fork = Expand_Basic::Fork;
    
    std::unordered_map<NodeRef, NodeRef> public_map; /// map in refs to public out refs. Each in ref should have exactly one out ref.

    template <typename OutputIt>
    OutputIt transition(const Fork& fork_src, NodeRef in_src, NodeRef in_dst, OutputIt out) const {
        std::vector<Fork> forks_dst;
        Expand_Basic::transition(fork_src, in_src, in_dst, std::back_inserter(forks_dst));
        
        // if branch, then reset speculation depths
        if (cfg.po.fwd.at(in_src).size() > 1) {
            for (Fork& fork_dst : forks_dst) {
                fork_dst.spec_depth = 1;
            }
        }
        
        // copy forks
        return std::copy(forks_dst.begin(), forks_dst.end(), out);
    }

    
    NodeRef merge(const Fork& fork, NodeRef in_ref, NodeRef out_ref) {
        if (fork.spec_depth <= num_specs) {
            // private
            return out_ref;
        } else {
            // public
            // NOTE: This will insert node if missing and tell caller to instantiate a fresh one.
            const auto res = public_map.emplace(in_ref, out_ref);
            return res.first->second;
        }
    }
    
    template <typename... Args>
    Expand_SpectreV1(Args&&... args): Expand_Basic(std::forward<Args>(args)...) {}
};

struct Expand_SpectreV4: Expand_Basic {
    std::unordered_map<NodeRef, NodeRef> public_map;
    
    struct Fork: Expand_Basic::Fork {
        bool always_speculative;
        
        Fork(const Expand_Basic::Fork& basic, bool always_speculative): Expand_Basic::Fork(basic), always_speculative(always_speculative) {}
        
        bool consistent(unsigned num_specs) const {
            if (always_speculative && spec_depth > num_specs) { return false; }
            return true;
        }
    };
    
    template <typename OutputIt>
    OutputIt transition(const Fork& fork_src, NodeRef in_src, NodeRef in_dst, OutputIt out) const {
        assert(fork_src.consistent(num_specs));
        
        std::vector<Expand_Basic::Fork> basic_forks_dst;
        Expand_Basic::transition(fork_src, in_src, in_dst, std::back_inserter(basic_forks_dst));
        
        for (const Expand_Basic::Fork& basic_fork_dst : basic_forks_dst) {
            if (fork_src.always_speculative) {
                // at most one fork
                const Fork fork_dst {basic_fork_dst, true};
                if (fork_dst.consistent(num_specs)) {
                    *out++ = fork_dst;
                }
            } else {
                *out++ = Fork {basic_fork_dst, false}; // non-speculative fork
                const auto& in_dst_node = cfg.lookup(in_dst);
                if (in_dst_node.may_read() && !std::holds_alternative<Exit>(in_dst_node.v)) {
                    *out++ = Fork {Expand_Basic::Fork {1}, true}; // speculative fork
                    // NOTE: this assumes that num_specs >= 1.
                }
            }
        }
        
        return out;
    }
    
    NodeRef merge(const Fork& fork, NodeRef in_ref, NodeRef out_ref) {
        if (fork.always_speculative) {
            return out_ref;
        } else {
            const auto res = public_map.emplace(in_ref, out_ref);
            return res.first->second;
        }
    }
    
    Fork init() const {
        return Fork {Expand_Basic::init(), false};
    }
    
    template <typename... Args>
    Expand_SpectreV4(Args&&... args): Expand_Basic(std::forward<Args>(args)...) {}
};
