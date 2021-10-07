#pragma once

#include <map>
#include <utility>
#include <optional>

#include <llvm/IR/Instructions.h>

#include "noderef.h"

class CFG;
struct SpeculationPrimitive;

/* Captures all information about a particular kind of speculation primitive.
 * Needs to be sufficiently general to support instantiating new forks under particular conditions.
 *
 * INFORMATION THAT SHOULD BE PROVIDED:
 * - Given an instruction, can the successor node(s) be speculative?
 *   - For branch primitives, the answer is yes iff the node is a branch.
 *   - For memory access primitives, the answer is yes to all successors that are memory accesses.
 * - How many of each successor node should be instantiated?
 * can_speculate() --
 *
 * Other data that should be included:
 * - Some speculative forks MUST be speculative. Add flag to force this.
 */

struct Fork {
    bool always_speculative;
};

struct SpeculationInfo {
    struct PrimitiveTFOInfo {
        std::vector<Fork> forks;
    };
    
    const CFG& po;
    std::map<std::pair<NodeRef, NodeRef>, PrimitiveTFOInfo> primitive_tfos;
 
    SpeculationInfo(const CFG& po);
    
    void add(const SpeculationPrimitive& primitive);
};

struct SpeculationPrimitive {
    // TODO: improve return type?
    using Result = std::optional<std::vector<Fork>>;
    virtual Result can_tfo(const llvm::Instruction *src, const llvm::Instruction *dst) const = 0;
    virtual ~SpeculationPrimitive() {}
};

struct BranchPrimitive: public SpeculationPrimitive {
    virtual Result can_tfo(const llvm::Instruction *src, const llvm::Instruction *dst) const override;
};

/* Any instruction preceding a memory access instruction branches to two parallel copies.
 */
struct AddrSpecPrimitive: public SpeculationPrimitive {
    virtual Result can_tfo(const llvm::Instruction *src, const llvm::Instruction *dst) const override;
};
