#pragma once

#include <variant>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>

#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Analysis/LoopInfo.h>

#include "noderef.h"
#include "binrel.h"
#include "lcm.h"

class AEGPO2 {
public:
   using NodeRef = std::size_t;

   struct Node {
      using Variant = std::variant<Entry, Exit, const llvm::Instruction *>;
      Variant v;

      unsigned func_id = 0; // invalid if == 0
      unsigned loop_id = 0; // invalid if == 0
      
      const Variant& operator()() const { return v; }
      Variant& operator()() { return v; }

      static Node make_entry() { return Node {Entry {}}; }
      static Node make_exit() { return Node {Exit {}}; }
      static Node make(const llvm::Instruction *I) { return Node {I}; }
   };

   using Rel = binrel<NodeRef>;
   Rel po; // simple po

   using NodeRefSet = std::unordered_set<NodeRef>;
   using LFMap = std::unordered_map<NodeRef, std::shared_ptr<NodeRefSet>>;
   LFMap funcs;
   LFMap loops;

   NodeRef entry;
   NodeRef exit;
   
   explicit AEGPO2(llvm::Function& F, unsigned num_unrolls = 2):
      F(F),
      num_unrolls(num_unrolls) {
      if (num_unrolls == 0) {
         throw std::invalid_argument("positive number of loop unrolls required");
      }
      construct();
   }

   llvm::raw_ostream& dump(llvm::raw_ostream& os) const;
   void dump_graph(const std::string& path) const;

   Node& lookup(NodeRef ref) { return nodes.at(ref); }
   const Node& lookup(NodeRef ref) const { return nodes.at(ref); }
   
   // TODO: these should be private private:
   llvm::Function& F;
   const unsigned num_unrolls;
   std::vector<Node> nodes; //

   void dump_loops(llvm::raw_ostream& os) const { dump_lf_map(os, loops); }
   void dump_funcs(llvm::raw_ostream& os) const { dump_lf_map(os, funcs); }

private:

   void dump_lf_map(llvm::raw_ostream& os, const LFMap& map) const;

   void construct();

   struct Port {
      NodeRef entry;
      std::unordered_multimap<const llvm::BasicBlock *, NodeRef> exits;
   };

   template <typename OutputIt>
   void construct_instruction(const llvm::Instruction *I, Port& port, OutputIt out);

   template <typename OutputIt>
   void construct_call(const llvm::CallBase *C, Port& port, OutputIt out);

   template <typename OutputIt>
   void construct_block(const llvm::BasicBlock *B, Port& port, OutputIt out);

   void construct_loop(const llvm::Loop *L, Port& port);
   void construct_function(llvm::Function *F, Port& port);
   
   struct LoopForest {
      const llvm::BasicBlock *entry;
      std::vector<const llvm::BasicBlock *> exits;
      std::vector<const llvm::BasicBlock *> blocks;
      std::vector<const llvm::Loop *> loops;
   };
   template <typename OutputIt>
   void construct_loop_forest(const LoopForest *LF, Port& port, OutputIt out);
   
   template <typename InputIt>
   void connect(InputIt src_begin, InputIt src_end, NodeRef dst) {
      for (auto src_it = src_begin; src_it != src_end; ++src_it) {
         add_edge(src_it->second, dst);
      }
   }
   
   void add_edge(NodeRef src, NodeRef dst) {
      po.insert(src, dst);
   }
   NodeRef add_node(const Node& node) {
      const NodeRef res = nodes.size();
      nodes.push_back(node);
      po.add_node(res);
      return res;
   }
   NodeRef add_node(Node&& node) {
      const NodeRef res = nodes.size();
      nodes.push_back(node);
      po.add_node(res);
      return res;
   }

   void erase_edge(NodeRef src, NodeRef dst) { po.erase(src, dst); }

   void prune();

   void add_lf_set(const NodeRefSet& set, LFMap& map);
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
