/** \dir src/cfg
 */

#pragma once

#include <variant>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <optional>
#include <iostream>

#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/LoopInfo.h>
#include <llvm/Analysis/AliasAnalysis.h>

#include "binrel.h"
#include "lcm.h"
#include "hash.h"
#include "noderef.h"
#include "util/output.h"



/* How to determine if you can use AA results to check whether A aliases B:
 * - Must be the case that A.func_id == B.func_id
 * - Must be the case that A.loop_id is a prefix of B.loop_id or vice versa.
 *
 */

/**
 * Control-flow graph representation during pre-processing of the function being analyzed.
 * All CFGs should have the property of being loop-free.
 * Multiple passes are performed on the original CFG before it is transformed into the AEG for analysis.
 */
class CFG {
public:
    using FuncID = unsigned; /*!< Globally (in the entire CFG) uniquely identifies current function instantiation (used for distinguishing multiply inlined functions) */
    using LoopID = unsigned; /*!< Locally (only in the current function/loop context) uniquely identifies a loop iteration (used for distinguishing different iterations of an unrolled loop) */
    
    /** A unique identifier for a node in the CFG.
     * This captures the current loop-nest context and function context.
     * This identifier is essential to correctly interpret the results of some LLVM analysis, e.g. LLVM alias analysis.
     */
    struct ID {
        std::vector<FuncID> func;
        std::vector<LoopID> loop;
        bool operator==(const ID& other) const {
            return func == other.func && loop == other.loop;
        }
        bool operator<(const ID& o) const noexcept {
            if (func == o.func) {
                return loop < o.loop;
            } else {
                return func < o.func;
            }
        }
    };
    
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
    using Rel = binrel<NodeRef>;
    Rel po;
    NodeRef entry;
    NodeRefSet exits;
    const unsigned num_specs;
    
    std::vector<Node> nodes;
    
    Node& lookup(NodeRef ref) { return nodes.at(ref); }
    const Node& lookup(NodeRef ref) const { return nodes.at(ref); }
    
    llvm::raw_ostream& dump(llvm::raw_ostream& os) const;
    
    void dump_graph(const std::string& path) const {
        po.group().dump_graph(path, [&] (auto& os, const auto& group) {
            for (NodeRef ref : group) {
                os << ref << " " << lookup(ref) << "\n";
            }
        });
    }
    
    std::string function_name() const;
    
    std::size_t size() const { return nodes.size(); }
    
    static bool llvm_alias_valid(const ID& a, const ID& b);
    static bool llvm_alias_valid(const Node& a, const Node& b);
    bool llvm_alias_valid(NodeRef a, NodeRef b) const { return llvm_alias_valid(lookup(a), lookup(b)); }
    
private:
    mutable std::optional<NodeRefVec> cached_postorder;
    bool postorder_rec(NodeRefSet& done, NodeRefVec& order, NodeRef ref) const;
public:
    const NodeRefVec& postorder() const;
    using ReversePostorderIteratorRange = llvm::iterator_range<NodeRefVec::const_reverse_iterator>;
    auto reverse_postorder() const {
        return llvm::iterator_range<NodeRefVec::const_reverse_iterator> {postorder().rbegin(), postorder().rend()};
    }
    
    CFG(unsigned num_specs): num_specs(num_specs) {}
    
    bool is_block_entry(NodeRef ref) const;
    bool is_block_exit(NodeRef ref) const;
    std::optional<NodeRef> get_block_successor(NodeRef ref) const;
    
    bool is_ancestor(NodeRef parent, NodeRef child) const;
    
protected:
    
    NodeRef add_node(const Node& node);
    
    void add_edge(NodeRef src, NodeRef dst) {
        po.insert(src, dst);
    }
    
    void erase_edge(NodeRef src, NodeRef dst) {
        po.erase(src, dst);
    }
    
    void prune();
    
    bool is_block_boundary(NodeRef ref, const Rel::Map& fwd, const Rel::Map& rev) const;
    
public:
    struct Translations {
        struct Key {
            std::vector<FuncID> id;
            const llvm::Value *V;
            Key(const std::vector<FuncID>& id, const llvm::Value *V): id(id), V(V) {}
            struct Hash {
                std::size_t operator()(const Key& key) const {
                    return llvm::hash_value(std::make_pair(std::hash<std::vector<FuncID>>()(key.id), key.V));
                }

            };
            bool operator==(const Key& other) const { return id == other.id && V == other.V; }
            bool operator<(const Key& other) const {
                if (V < other.V) {
                    return true;
                } else if (V == other.V) {
                    return id < other.id;
                } else {
                    return false;
                }
            }
        };
        
        struct Value {
            std::vector<FuncID> id;
            using ValueSet = std::unordered_set<const llvm::Value *>;
            ValueSet Vs;
            Value(const std::vector<FuncID>& id): id(id) {}
            Value(const std::vector<FuncID>& id, const ValueSet& Vs): id(id), Vs(Vs) {}
            bool operator==(const Value& other) const {
                return id == other.id && Vs == other.Vs;
            }
        };
        
#if 1
        using Map = std::unordered_map<Key, Value, Key::Hash>;
#elif 0
        using Map = std::map<Key, Value>;
#endif
        
        Map map;
        
        template <typename OutputIt>
        OutputIt lookup(const Key& key, OutputIt out) const {
            std::vector<Key> todo {key};
            std::unordered_set<Key, Key::Hash> seen;
            std::unordered_set<Key, Key::Hash> done;
            
            while (!todo.empty()) {
                const Key key = todo.back();
                todo.pop_back();
                if (seen.insert(key).second) {
                    const auto it = map.find(key);
                    if (it == map.end()) {
                        done.insert(key);
                    } else {
                        for (const llvm::Value *V : it->second.Vs) {
                            todo.emplace_back(it->second.id, V);
                        }
                    }
                }
            }
            
            return std::copy(done.begin(), done.end(), out);
        }
        
        bool operator==(const Translations& other) const {
            return map == other.map;
        }
    };
    
    Translations translations;
    
    bool operator==(const CFG& other) const {
        return nodes == other.nodes && po == other.po && translations == other.translations;
    }
    
    struct partial_order;
    partial_order make_partial_order() const;
    
    bool may_introduce_speculation(NodeRef ref) const;
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const CFG::Node& node);

/* Functions and loops may have multiple exits.
 * For functions, these are return instructions.
 * For loops, these are branch instructions.
 *
 * Loops can be abstracted as a basic block that can go to any number of successors.
 * Then what we do is replace that loop with its unrolled version.
 * So really what we're doing is replacing loop nodes and function nodes in the CFG with
 * their unrolled and inlined counterparts.
 *
 * Actually, BasicBlocks are even abstractions of instructions.
 * We should construct the PO graph using a depth-first recursive approach.
 * construct_function()
 * construct_loop()
 * construct_block()
 * construct_instruction()
 * All of these return the entering instruction and set of exiting instructions.
 * For all of these, they must have exactly one entering instruction and any number of exiting
 * instructions (for now, we can assume >= 1 exits).
 */

namespace std {
template <>
struct hash<CFG::ID> {
    std::size_t operator()(const CFG::ID& id) const {
        return hash_ordered_tuple(id.func, id.loop);
    }
};
}

std::ostream& operator<<(std::ostream& os, const CFG::ID& id);
std::ostream& operator<<(std::ostream& os, const CFG::Node::Call& call);
std::ostream& operator<<(std::ostream& os, const CFG::Node::Variant& v);

// TODO: optimize. Perhaps use cache of nodes already visited
struct CFG::partial_order {
    const CFG& cfg;
    
    /** Return whether \p a is a parent of \p b. */
    bool operator()(NodeRef a, NodeRef b) const {
        for (const auto b_pred : cfg.po.rev.at(b)) {
            if (a == b_pred) { return true; }
            if ((*this)(a, b_pred)) { return true; }
        }
        return false;
    }
};

inline CFG::partial_order CFG::make_partial_order() const {
    return partial_order {.cfg = *this};
}




llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const CFG::Translations::Key& key);
llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const CFG::Translations::Value& value);
