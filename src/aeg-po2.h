#pragma once

#include <variant>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <optional>

#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/LoopInfo.h>

#include "binrel.h"
#include "lcm.h"
#include "hash.h"

/* How to determine if you can use AA results to check whether A aliases B:
 * - Must be the case that A.func_id == B.func_id
 * - Must be the case that A.loop_id is a prefix of B.loop_id or vice versa.
 * 
 */

class AEGPO2 {
public:
   using NodeRef = std::size_t;
   using NodeRefSet = std::unordered_set<NodeRef>;
   using NodeRefVec = std::vector<NodeRef>;
   using FuncID = unsigned;
   using LoopID = unsigned;

   // TODO: Need to use stack for loops too
   struct ID {
      FuncID func;
      std::vector<LoopID> loop;
      bool operator==(const ID& other) const {
         return func == other.func && loop == other.loop;
      }
   };
   
   struct Node {
      using Variant = std::variant<Entry, Exit, const llvm::Instruction *>;
      
      Variant v;
      std::optional<ID> id;
      std::unordered_map<const llvm::Value *, NodeRefSet> refs;
      
      const Variant& operator()() const { return v; }
      Variant& operator()() { return v; }

      Node() {} // TODO: remove this?
      template <typename Arg>
      explicit Node(const Arg& arg): v(arg) {}
      static Node make_entry() { return Node {Entry {}}; }
      static Node make_exit() { return Node {Exit {}}; }
      static Node make(const llvm::Instruction *I) { return Node {I}; }
   };
   using Rel = binrel<NodeRef>;
   Rel po;
   NodeRef entry;
   NodeRef exit;

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
   
protected:
   NodeRef add_node(const Node& node) {
      const NodeRef ref = size();
      nodes.push_back(node);
      po.add_node(ref);
      return ref;
   }   
   
   void add_edge(NodeRef src, NodeRef dst) {
      po.insert(src, dst);
   }

   void erase_edge(NodeRef src, NodeRef dst) {
      po.erase(src, dst);
   }

   void prune();   
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO2::Node& node);


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
   struct hash<AEGPO2::ID> {
      std::size_t operator()(const AEGPO2::ID& id) const {
         return hash_ordered_tuple(id.func, id.loop);
      }
   };
}
