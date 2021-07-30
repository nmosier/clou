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

/* TODO
 * [ ] Clear out unused functions
 * [ ] Merge nodes into basic blocks in output.
 */

class AEGPO {
public:
   struct Node {
      const llvm::Instruction *I;

      llvm::raw_ostream& dump(llvm::raw_ostream& os, const char *special) const {
         if (I) {
            os << *I;
         } else {
            os << special;
         }
         return os;
      }
   };

   Node *entry;
   using Rel = binrel<Node *>;
   Rel po; // simple po
   Rel po_trans; // transitive po

   AEGPO(): entry(new Node {nullptr}) {
      /* insert entry and exit into po */
      nodes.push_back(std::unique_ptr<Node> {entry});
      po.add_node(entry);
   }

   void construct2(const CFG2& cfg, unsigned num_unrolls = 2);
   
   llvm::raw_ostream& dump(llvm::raw_ostream& os) const;
   void dump_graph(const std::string& path) const;

private:
   std::vector<std::unique_ptr<Node>> nodes;
   using Loop = std::unordered_set<const llvm::Instruction *>;
   using Loops = std::unordered_set<Loop>;
   Loops loops;
   
   void add_edge(Node *src, Node *dst);

   template <typename InputIt, typename OutputIt>
   void predecessor_nodes(InputIt begin, InputIt end, OutputIt out) const;

   template <typename Container>
   void predecessor_nodes(Container& container) const;

   size_t depth(Node *node) const;

   using MergeMap = std::unordered_map<const llvm::Instruction *, std::unordered_set<Node *>>;
   using RepMap = std::unordered_map<const llvm::Instruction *, unsigned>;
   using NodeVec = std::vector<Node *>;

   template <typename OutputIt>
   void construct2_rec(const CFG2& cfg, unsigned num_unrolls, Node *node, MergeMap& merge_map,
                       const RepMap& reps_, NodeVec trace, OutputIt& out);

   bool is_ancestor(Node *child, Node *parent) const;
   bool is_ancestor_a(Node *child, Node *parent) const;
   bool is_ancestor_b(Node *child, Node *parent) const;

   bool is_sibling(Node *a, Node *b) const;

   unsigned max_reps(Node *node) const {
      RepMap reps;
      return max_reps(node, reps);
   }
   unsigned max_reps(Node *node, RepMap reps) const;

   size_t node_id(const Node *node) const {
      return std::find_if(nodes.begin(), nodes.end(), [node] (const auto& ptr) {
         return ptr.get() == node;
      }) - nodes.begin();
   }

   void prune();
   void erase(Node *node);

   using BB = std::vector<Node *>;
   binrel<BB> get_bbs() const;
   BB get_bb(Node *node) const;

   bool is_exit(Node *node) const;
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO& aeg);
