#pragma once

#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <ostream>

#include <llvm/IR/Instruction.h>
#include <llvm/Support/raw_ostream.h>

#include "util.h"
#include "binrel.h"

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

   void construct(const CFG& cfg);

   
   llvm::raw_ostream& dump(llvm::raw_ostream& os) const;

private:
   std::vector<std::unique_ptr<Node>> nodes;
   
   void add_edge(Node *src, Node *dst);
#if 0
   bool check_loop(Node *node) const;
   bool check_loop_i(Node *node, size_t loopsize) const;
   bool check_loop_i_rec(std::vector<Node *>& trace, size_t loopsize) const;
#else

#endif

   template <typename InputIt, typename OutputIt>
   void predecessor_nodes(InputIt begin, InputIt end, OutputIt out) const;

   template <typename Container>
   void predecessor_nodes(Container& container) const;

   size_t depth(Node *node) const;

   using MergeMap = std::unordered_map<const llvm::Instruction *, std::unordered_set<Node *>>;
   using RepMap = std::unordered_map<const llvm::Instruction *, unsigned>;
   void construct(const CFG& cfg, Node *node, MergeMap& merge_map, RepMap reps);

   bool is_ancestor(Node *child, Node *parent) const;
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO& aeg);
