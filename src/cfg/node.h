#pragma once

#include <iostream>
#include <variant>

#include <llvm/IR/Instructions.h>

#include "lcm.h"
#include "cfg/cfg.h" // TODO: remove this



namespace cfg {

/** A node in the control flow graph. A node can represent one of the following:
 * - Entry: program entry node (top). Each CFG contains exactly one of these. It does not have a corresponding llvm::Instruction.
 * - Exit: program exit node (bottom). Each CFG contains at least one of these. It does not have a corresponding llvm::Instruction.
 * - llvm::Instruction: an instruction. There is possibly a 1-to-many correspondence from these nodes to LLVM-IR instructions due to function inlining and loop unrolling.
 * - Node::Call: the evaluation of a single pointer argument of an external call.
 */
struct Node {
    struct Call {
        const llvm::CallBase *C;
        const llvm::Value *arg;
        bool operator==(const Call& other) const {
            return C == other.C && arg == other.arg;
        }
    };
    using Variant = std::variant<Entry, Exit, const llvm::Instruction *, Call>;
    
    Variant v;
    using ID = CFG::ID;
    std::optional<ID> id;
    std::unordered_map<const llvm::Value *, NodeRefSet> refs;
    
    const Variant& operator()() const { return v; }
    Variant& operator()() { return v; }
    
    Node() {} // TODO: remove this?
    
    template <typename Arg>
    explicit Node(const Arg& arg, std::optional<ID> id): v(arg), id(id) {}
    
    static Node make_entry() { return Node {Entry {}, std::nullopt}; }
    static Node make_exit() { return Node {Exit {}, std::nullopt}; }
    
    bool operator==(const Node& other) const {
        return v == other.v && id == other.id && refs == other.refs;
    }
    
    bool may_read() const;
};

std::ostream& operator<<(std::ostream& os, const Node::Call& call);
std::ostream& operator<<(std::ostream& os, const Node::Variant& v);



}
