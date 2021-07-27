#include <cassert>
#include <algorithm>
#include <functional>

#include "aeg2.h"
#include "util.h"

#define AEG AEG2

void AEG::construct(const llvm::Function& F) {
   BinaryInstRel preds, succs;
   predecessor_map(F, preds);
   successor_map(F, succs);

   
}

void AEG::Node::construct(const BinaryInstRel& preds, const BinaryInstRel& succs) {
   // NOTE: Expect the `next` relation to be empty. `prev` may or may not be empty.
   assert(next.empty());
   
   /* The first branch is constructed specially: */
   
}

size_t AEG::Node::depth() const {
   size_t depth = 0;
   for (const auto& pair : prev) {
      depth = std::max(depth, pair.first->depth() + 1);
   }
   return depth;
}

bool AEG::Node::check_loop() const {
   /* calculate depth (distance to entry) */
   const size_t depth_ = depth();

   /* Algorithm: 
    * For each loop size i,
    *  Initialize nodes1 with current node
    *  Initialize nodes2 with nodes that are `i` edges away
    *  
    */

   const NodeSet nodes1 {this};
   NodeSet nodes2 {this};
   size_t loopsize = 0;
   goto entry;
   while (loopsize < depth_ / 2) {
      /* check loop for specific value */
      // we will be following up a set of paths 
      

   entry:
      /* graduate nodes 2 once */
      {
         NodeSet tmp;
         graduate_nodes(nodes2.begin(), nodes2.end(), std::back_inserter(tmp));
         nodes2 = std::move(tmp);
      }
      
      ++loopsize;
   }
   
}

bool AEG::Node::check_loop_i(size_t loopsize, NodeSet nodes1, NodeSet nodes2) const {
   using Map = std::unordered_map<const Node *, NodeSet>; // map tier 1 node to tier 2 nodes

   /* initialize map */
   Map map;
   
   
   for (size_t i = 0; i < loopsize; ++i) {
      /* check for matches */
      NodeSet intersection;
      std::set_intersection(nodes1.begin(), nodes1.end(), nodes2.begin(), nodes2.end(),
                            std::back_inserter(intersection));

      /* If no match, then all paths dead */
      if (intersection.empty()) {
         return false;
      }

      graduate_nodes(nodes1);
      graduate_nodes(nodes2);
   }
}

#undef AEG
