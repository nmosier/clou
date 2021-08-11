#pragma once

#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <ostream>

#include <llvm/IR/Instruction.h>
#include <llvm/Support/raw_ostream.h>

#include "util.h"
#include "cfg.h"
#include "binrel.h"
#include "noderef.h"

/* TODO
 * [ ] Clear out unused functions
 * [ ] Merge add_node <- po.add_node
 */

class AEGPO {
public:
   using NodeRef = ::NodeRef<AEGPO>;
#if 1
   struct Node {
      CFG::NodeRef cfg_ref;
      Node() {}
      Node(CFG::NodeRef cfg_ref): cfg_ref(cfg_ref) {}
      void dump(llvm::raw_ostream& os, const CFG& cfg) const;
   };
#else
   using Entry = CFG::Entry;
   using Exit = CFG::Exit;
   using Node = CFG::Node;
#endif
   
   static inline const NodeRef entry {0};

   using Rel = binrel<NodeRef>;
   Rel po; // simple po
   
   explicit AEGPO(const CFG& cfg): cfg(cfg), nodes({Node {CFG::entry}}) {
      po.add_node(entry);
   }

   void construct2(unsigned num_unrolls = 2);
   
   llvm::raw_ostream& dump(llvm::raw_ostream& os) const;
   void dump_graph(const std::string& path) const;

   const Node& lookup(NodeRef ref) const { return nodes.at((unsigned) ref); }
   
private:
   const CFG& cfg;
   std::vector<Node> nodes;

   void add_edge(NodeRef src, NodeRef dst);

   template <typename InputIt, typename OutputIt>
   void predecessor_nodes(InputIt begin, InputIt end, OutputIt out) const;

   template <typename Container>
   void predecessor_nodes(Container& container) const;

   using MergeMap = std::unordered_map<CFG::NodeRef, std::unordered_set<NodeRef>>;
   using RepMap = std::unordered_map<CFG::NodeRef, unsigned>;
   using NodeVec = std::vector<NodeRef>;
   struct ConstructionTask {
      NodeRef node;
      RepMap reps;
      NodeVec trace;
   };

   template <typename OutputIt>
   OutputIt construct2_rec(unsigned num_unrolls,
                           MergeMap& merge_map, ConstructionTask& task, OutputIt out);

   bool is_ancestor(NodeRef child, NodeRef parent) const;
   bool is_ancestor_a(NodeRef child, NodeRef parent) const;
   bool is_ancestor_b(NodeRef child, NodeRef parent) const;
   bool is_ancestor_d(NodeRef child, NodeRef parent) const;
   bool is_ancestor_e(NodeRef child, NodeRef parent) const;
   std::optional<NodeRef> is_any_not_ancestor(NodeRef child, Rel::Set parents) const;
   
   void prune();
   void erase(NodeRef node);
   using BB = std::vector<NodeRef>;
   binrel<BB> get_bbs() const;
   BB get_bb(Node *node) const;
   // bool is_exit(const Node& node) const;

   bool is_mergable(NodeRef node, NodeRef merge_candidate, const NodeVec& trace) const;

   template <typename... Ts>
   NodeRef add_node(Ts&&... ts) {
      const NodeRef ref {static_cast<unsigned>(nodes.size())};
      nodes.emplace_back(std::forward<Ts>(ts)...);
      po.add_node(ref);
      llvm::errs() << "adding node " << ref << "\n";
      return ref;
   }

   bool is_entry(NodeRef node) const { return node == entry; }
   bool is_exit(NodeRef node) const {
      return cfg.is_exit(lookup(node).cfg_ref);
   }

   friend class AEG;
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO& aeg);
