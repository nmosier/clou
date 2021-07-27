#pragma once

#include <memory>
#include <vector>
#include <set>

#include <llvm/IR/Instruction.h>

#include "util.h"

#define AEG AEG2

// TODO: Is it better to have a pointer to a list or a list of pointers?
// TODO: Edges should be simple list of pairs, not map.

struct AEGEdges;

class AEG {
public:
   struct Node;
   struct Edge {};

   using Edges = std::unordered_map<std::shared_ptr<Node>, std::vector<Edge>>;
   
   class Node {
   public:
      const llvm::Instruction *inst;
      Edges next;
      Edges prev;

      Node(const llvm::Instruction *inst): inst(inst) {} 
      Node(const llvm::Instruction *inst, const Edges& next, const Edges& prev):
         inst(inst), next(next), prev(prev) {}

      void construct(const BinaryInstRel& preds, const BinaryInstRel& succs);

      Node(Node&& other):
         inst(other.inst), next(std::move(other.next)), prev(std::move(other.prev)) {}

   private:
      using NodeSet = std::set<const Node *>;
      
      bool check_loop() const;
      bool check_loop_i(size_t loopsize, NodeSet nodes1, NodeSet nodes2) const;
      
      template <typename InputIt, typename OutputIt>
      static void graduate_nodes(InputIt begin, InputIt end, OutputIt out);
      
      template <typename Container>
      static void graduate_nodes(Container& container) {
         Container tmp;
         graduate_nodes(container.begin(), container.end(), std::back_inserter(tmp));
         container = std::move(tmp);
      }
      
      size_t depth() const;
   };

   Node entry;
   Node exit;

   AEG(): entry(nullptr), exit(nullptr) {}

   void construct(const llvm::Function& F);

private:
};

#if 0
template <typename InputIt, typename OutputIt>
void AEG::Node::graduate_nodes(InputIt begin, InputIt end, OutputIt out) {
   for (auto it = begin; it != end; ++it) {
      for (const auto& prev_pair : node1->prev) {
         *out++ = prev_pair.first;
      }
   }
}
#endif

#undef AEG
