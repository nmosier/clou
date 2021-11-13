#pragma once

#include <vector>
#include <unordered_map>
#include <map>

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
    
    template <typename Expand>
    void construct(const CFG& in, Expand& expand);
    
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
    
    template <typename Expand>
    void construct_partial(const CFG& in, Expand& expand);
    using RefMap1 = std::unordered_map<Translations::Key, NodeRefSet, Translations::Key::Hash>;
    
    void resolve_single_ref(const llvm::Instruction *I, const llvm::Value *V, const CFG &in, std::vector<RefMap1>& maps, CFG::Node &node, NodeRef ref);
    
    void resolve_refs(const CFG& in);
};

/** Basic no-op expansion primitive. This simply increments the speculation depth along the path and outputs a new fork in a 1-1 correspondence to the original CFG. */
struct Expand_Basic {
    const CFG& cfg; /// input CFG
    unsigned num_specs; /// max speculation depth

    Expand_Basic(const CFG& cfg, unsigned num_specs): cfg(cfg), num_specs(num_specs) {}
    
    /** Basic fork record, which only tracks speculation depth. */
    struct Fork {
        unsigned spec_depth;
        
        virtual Option can_arch() const {
            return Option::MAY;
        }
        
        virtual Option can_trans(unsigned num_specs) const {
            if (spec_depth <= num_specs) {
                return Option::MAY;
            } else {
                return Option::NO;
            }
        }
        
        Fork(unsigned spec_depth): spec_depth(spec_depth) {}
        
        virtual ~Fork() {}
    };
    
    /** Transition function. Given a current fork and in noderef, output list of successor forks to process.
     */
    template <typename OutputIt>
    OutputIt transition(const Fork& fork_src, NodeRef in_src, NodeRef in_dst, OutputIt out) const {
        const unsigned new_spec_depth = std::min(fork_src.spec_depth, num_specs) + 1;
        *out++ = Fork(new_spec_depth);
        return out;
    }
    
    /** Returns initial fork, which is non-speculative. */
    Fork init() const {
        return Fork(num_specs + 1);
    }
};


/** Spectre-V1 expansion primitive. This is similar to the Expand_Basic primitive, but it resets speculation depth after all branches. */
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

    NodeRef merge(const Fork& fork, NodeRef in_ref, NodeRef out_ref);
    
    template <typename... Args>
    Expand_SpectreV1(Args&&... args): Expand_Basic(std::forward<Args>(args)...) {}
};


/** Spectre-V4 expansion primitive. This expands non-speculative loads into a speculative and non-speculative version. */
struct Expand_SpectreV4: Expand_Basic {
    using Map = std::unordered_map<NodeRef, NodeRef>;
    Map nonspeculative_map, speculative_map;
    
    struct Fork: Expand_Basic::Fork {
        bool always_speculative;
        
        Fork(const Expand_Basic::Fork& basic, bool always_speculative): Expand_Basic::Fork(basic), always_speculative(always_speculative) {}
        
        bool consistent(unsigned num_specs) const {
            if (always_speculative && spec_depth > num_specs) { return false; }
            return true;
        }
        
        virtual Option can_arch() const override {
            if (always_speculative) {
                return Option::NO;
            } else {
                return Option::MAY;
            }
        }
        
        virtual Option can_trans(unsigned num_specs) const override {
            if (always_speculative) {
                return Option::MAY;
            } else {
                return Option::NO;
            }
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
    
    NodeRef merge(const Fork& fork, NodeRef in_ref, NodeRef out_ref);
    
    Fork init() const {
        return Fork {Expand_Basic::init(), false};
    }
    
    template <typename... Args>
    Expand_SpectreV4(Args&&... args): Expand_Basic(std::forward<Args>(args)...) {}
};
