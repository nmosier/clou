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

#include "binrel.h"
#include "lcm.h"
#include "hash.h"
#include "noderef.h"

/* How to determine if you can use AA results to check whether A aliases B:
 * - Must be the case that A.func_id == B.func_id
 * - Must be the case that A.loop_id is a prefix of B.loop_id or vice versa.
 * 
 */

class AEGPO {
public:
   using FuncID = unsigned;
   using LoopID = unsigned;

   // TODO: Need to use stack for loops too
   struct ID {
      FuncID func = 0;
      std::vector<LoopID> loop;
      bool operator==(const ID& other) const {
         return func == other.func && loop == other.loop;
      }
   };
   
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
      
#if 0
      template <typename Arg>
      explicit Node(const Arg& arg): v(arg) {}
#endif
      static Node make_entry() { return Node {Entry {}, std::nullopt}; }
      static Node make_exit() { return Node {Exit {}, std::nullopt}; }
      // static Node make(const llvm::Instruction *I) { return Node {I}; }
       
       bool operator==(const Node& other) const {
           return v == other.v && id == other.id && refs == other.refs;
       }
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

   std::size_t size() const { return nodes.size(); }

   static bool alias_valid(const ID& a, const ID& b);
   static bool alias_valid(const Node& a, const Node& b);
   bool alias_valid(NodeRef a, NodeRef b) const { return alias_valid(lookup(a), lookup(b)); }

   template <typename OutputIt>
   void reverse_postorder(OutputIt out) const;

   template <typename OutputIt>
   void postorder(OutputIt out) const;

   AEGPO(unsigned num_specs): num_specs(num_specs) {}
    
    bool is_block_entry(NodeRef ref) const;
    bool is_block_exit(NodeRef ref) const;
    std::optional<NodeRef> get_block_successor(NodeRef ref) const;
   
protected:
   bool postorder_rec(NodeRefSet& done, NodeRefVec& order, NodeRef ref) const;
   
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
            FuncID id;
            const llvm::Value *V;
            Key(FuncID id, const llvm::Value *V): id(id), V(V) {}
            struct Hash {
                std::size_t operator()(const Key& key) const { return llvm::hash_value(std::make_tuple(key.id, key.V)); }
            };
            bool operator==(const Key& other) const { return id == other.id && V == other.V; }
        };
        
        struct Value {
            FuncID id;
            using ValueSet = std::unordered_set<const llvm::Value *>;
            ValueSet Vs;
            Value(FuncID id): id(id) {}
            Value(FuncID id, const ValueSet& Vs): id(id), Vs(Vs) {}
            bool operator==(const Value& other) const {
                return id == other.id && Vs == other.Vs;
            }
        };
        
        using Map = std::unordered_map<Key, Value, Key::Hash>;
        
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
    
    bool operator==(const AEGPO& other) const {
        return nodes == other.nodes && po == other.po && translations == other.translations;
    }
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO::Node& node);

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
   struct hash<AEGPO::ID> {
      std::size_t operator()(const AEGPO::ID& id) const {
         return hash_ordered_tuple(id.func, id.loop);
      }
   };
}


template <typename OutputIt>
void AEGPO::reverse_postorder(OutputIt out) const {
   std::unordered_set<NodeRef> done;
   std::vector<NodeRef> order;
   postorder_rec(done, order, entry);
   std::copy(order.rbegin(), order.rend(), out) ;
}

template <typename OutputIt>
void AEGPO::postorder(OutputIt out) const {
   std::unordered_set<NodeRef> done;
   std::vector<NodeRef> order;
   postorder_rec(done, order, entry);
   std::copy(order.begin(), order.end(), out);
}

std::ostream& operator<<(std::ostream& os, const AEGPO::ID& id);
std::ostream& operator<<(std::ostream& os, const AEGPO::Node::Call& call);
std::ostream& operator<<(std::ostream& os, const AEGPO::Node::Variant& v);
