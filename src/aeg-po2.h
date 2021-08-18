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

#include "noderef.h"
#include "binrel.h"
#include "lcm.h"

// TODO: Resign this so it's a pure variant.
// TODO: Maybe avoid templating nodes too; just use one node definition.
struct AEGPO_Node_Base {
   using Variant = std::variant<Entry, Exit, const llvm::Instruction *>;
   Variant v;
   std::optional<unsigned> func_id;
   std::vector<unsigned> loop_id;
   const Variant& operator()() const { return v; }
   Variant& operator()() { return v; }

   AEGPO_Node_Base() {} // TODO: remove this?
   template <typename Arg>
   AEGPO_Node_Base(const Arg& arg): v(arg) {}
   static AEGPO_Node_Base make_entry() { return AEGPO_Node_Base {Entry {}}; }
   static AEGPO_Node_Base make_exit() { return AEGPO_Node_Base {Exit {}}; }
   static AEGPO_Node_Base make(const llvm::Instruction *I) { return AEGPO_Node_Base {I}; }
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO_Node_Base& node);

/* How to determine if you can use AA results to check whether A aliases B:
 * - Must be the case that A.func_id == B.func_id
 * - Must be the case that A.loop_id is a prefix of B.loop_id or vice versa.
 * 
 */

class AEGPO2 {
public:
   using Node = AEGPO_Node_Base;
   using NodeRef = std::size_t;
   using Rel = binrel<NodeRef>;
   Rel po;
   NodeRef entry;
   NodeRef exit;

   std::vector<Node> nodes;

   Node& lookup(NodeRef ref) { return nodes.at(ref); }
   const Node& lookup(NodeRef ref) const { return nodes.at(ref); }

   using NodeRefSet = std::unordered_set<NodeRef>;

   llvm::raw_ostream& dump(llvm::raw_ostream& os) const;



   void dump_graph(const std::string& path) const {
      po.group().dump_graph(path, [&] (auto& os, const auto& group) {
         for (NodeRef ref : group) {
            os << ref << " " << lookup(ref) << "\n";
         }
      });
   }

   std::size_t size() const { return nodes.size(); }

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

   static bool alias_valid(const Node& a, const Node& b);

   void prune();   
};

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
