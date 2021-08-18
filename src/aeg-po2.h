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

   template <typename Arg>
   AEGPO_Node_Base(const Arg& arg): v(arg) {}
   static AEGPO_Node_Base make_entry() { return AEGPO_Node_Base {Entry {}}; }
   static AEGPO_Node_Base make_exit() { return AEGPO_Node_Base {Exit {}}; }
   static AEGPO_Node_Base make(const llvm::Instruction *I) { return AEGPO_Node_Base {I}; }
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO_Node_Base& node);

class AEGPO_Base_ {
public:
   using NodeRef = std::size_t;
   using Node = AEGPO_Node_Base;
   using Rel = binrel<NodeRef>;
   Rel po;
   NodeRef entry;
   NodeRef exit;

   std::vector<std::unique_ptr<Node>> nodes;
   std::size_t size() const { return nodes.size(); }
   Node& lookup(NodeRef ref) { return *nodes.at(ref); }
   const Node& lookup(NodeRef ref) const { return *nodes.at(ref); }

protected:
   void add_edge(NodeRef src, NodeRef dst) {
      po.insert(src, dst);
   }

   void erase_edge(NodeRef src, NodeRef dst) {
      po.erase(src, dst);
   }

   static bool alias_valid(const Node& a, const Node& b);

private:
};

template <typename NodeT>
class AEGPO_Base: public AEGPO_Base_ {
public:
   using Node = NodeT;
   
   Node& lookup(NodeRef ref) { return static_cast<Node&>(AEGPO_Base_::lookup(ref)); }
   const NodeT& lookup(NodeRef ref) const {
      return static_cast<const Node&>(AEGPO_Base_::lookup(ref));
   }

   void dump_graph(const std::string& path) const {
      po.group().dump_graph(path, [&] (auto& os, const auto& group) {
         for (NodeRef ref : group) {
            os << ref << " " << lookup(ref) << "\n";
         }
      });
   }

protected:
   NodeRef add_node(const Node& node) {
      const NodeRef ref = size();
      nodes.push_back(std::make_unique<Node>(node));
      po.add_node(ref);
      return ref;
   }
};

/* How to determine if you can use AA results to check whether A aliases B:
 * - Must be the case that A.func_id == B.func_id
 * - Must be the case that A.loop_id is a prefix of B.loop_id or vice versa.
 * 
 */

class AEGPO2: public AEGPO_Base<AEGPO_Node_Base> {
public:
   using NodeRef = std::size_t;

   using NodeRefSet = std::unordered_set<NodeRef>;
   using LFMap = std::unordered_map<NodeRef, std::shared_ptr<NodeRefSet>>;
   LFMap funcs;
   LFMap loops;

   explicit AEGPO2(llvm::Function& F, unsigned num_unrolls = 2):
      F(F),
      num_unrolls(num_unrolls) {
      if (num_unrolls == 0) {
         throw std::invalid_argument("positive number of loop unrolls required");
      }
      construct();
   }

   llvm::raw_ostream& dump(llvm::raw_ostream& os) const;

   // TODO: these should be private private:
   llvm::Function& F;
   const unsigned num_unrolls;

   void dump_loops(llvm::raw_ostream& os) const { dump_lf_map(os, loops); }
   void dump_funcs(llvm::raw_ostream& os) const { dump_lf_map(os, funcs); }

private:

   void dump_lf_map(llvm::raw_ostream& os, const LFMap& map) const;

   struct IDs {
      unsigned func_id = 0;
      unsigned next_loop_id = 0;
      std::vector<unsigned> loop_ids;
   };

   void construct();

   struct Port {
      NodeRef entry;
      std::unordered_multimap<const llvm::BasicBlock *, NodeRef> exits;
   };

   template <typename OutputIt>
   void construct_instruction(const llvm::Instruction *I, Port& port, OutputIt out, IDs& ids);

   template <typename OutputIt>
   void construct_call(const llvm::CallBase *C, Port& port, OutputIt out, IDs& ids);

   template <typename OutputIt>
   void construct_block(const llvm::BasicBlock *B, Port& port, OutputIt out, IDs& ids);

   void construct_loop(const llvm::Loop *L, Port& port, IDs& ids);
   void construct_function(llvm::Function *F, Port& port, IDs& ids);
   
   struct LoopForest {
      const llvm::BasicBlock *entry;
      std::vector<const llvm::BasicBlock *> exits;
      std::vector<const llvm::BasicBlock *> blocks;
      std::vector<const llvm::Loop *> loops;
   };
   template <typename OutputIt>
   void construct_loop_forest(const LoopForest *LF, Port& port, OutputIt out, IDs& ids);
   
   template <typename InputIt>
   void connect(InputIt src_begin, InputIt src_end, NodeRef dst) {
      for (auto src_it = src_begin; src_it != src_end; ++src_it) {
         add_edge(src_it->second, dst);
      }
   }
   
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
