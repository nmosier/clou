#include <cstdlib>
#include <algorithm>
#include <cassert>

#include "aeg-po.h"


/* BRAINSTORMING

   LOOP DETECTION / TERMINATION
   If you try to add an instruction a 3rd time, then stop -- you're in a loop.
   This only works for singly-nested loops. In doubly-nested loops, we would have 4 copies of the
   instruction. 
   Maintain a map from instructions to recurrence counts. Reset the recurrence count every time you
   stop a loop (recurrence count would be > 2). 
   
   BRANCHES
   Can merge with nodes that have a common anscestor but aren't an anscestor.
   

 */

void AEGPO::add_edge(Node *src, Node *dst) {
   llvm::errs() << "adding edge:     ";
   if (src->I) { llvm::errs() << *src->I; } else { llvm::errs() << "<entry>"; }
   if (dst->I) { llvm::errs() << *dst->I; } else { llvm::errs() << "<exit>"; }
   llvm::errs() << "\n";
   
   /* anything that passes into source and out of dst is now transitively connected. */
   po.insert(src, dst);

   po_trans.insert(src, dst);
   for (Node *dst_out : po.fwd[dst]) {
      po_trans.insert(src, dst_out);
   }
   for (Node *src_in : po.fwd[src]) {
      po_trans.insert(src_in, dst);
   }
   for (Node *src_in : po.fwd[src]) {
      for (Node *dst_out : po.fwd[dst]) {
         po_trans.insert(src_in, dst_out);
      }
   }
}

template <typename InputIt, typename OutputIt>
void AEGPO::predecessor_nodes(InputIt begin, InputIt end, OutputIt out) const {
   for (auto it = begin; it != end; ++it) {
      for (auto rev : po.rev[*it]) {
         *out++ = rev;
      }
   }
}

template <typename Container>
void AEGPO::predecessor_nodes(Container& container) const {
   Container tmp;
   predecessor_nodes(container.begin(), container.end(), std::back_inserter(tmp));
   container = std::move(tmp);
}

bool AEGPO::check_loop(Node *node) const {
   /* Temporary algorithm: check each individual path recursively */

   // Need to check whether there are two equal subgraphs that are connected
   // Alternatively, all paths from node must be a loop of length n for some n.
   // Try that formulation first.

   const size_t depth_ = depth(node) / 2;
   for (size_t loopsize = 1; loopsize <= depth_ / 2; ++loopsize) {
      if (check_loop_i(node, loopsize)) {
         return true;
      }
   }
   
   return false;
}

bool AEGPO::check_loop_i(Node *node, size_t loopsize) const {
   std::vector<Node *> trace {node};
   return check_loop_i_rec(trace, loopsize);
}

bool AEGPO::check_loop_i_rec(std::vector<Node *>& trace, size_t loopsize) const {
   assert(trace.size() > 0);
   if (trace.size() < loopsize) {
      /* recursively construct paths */
      Node *node = trace.back();
      trace.push_back(nullptr);
      for (Node *src : po.rev.at(node)) {
         trace.back() = src;
         if (!check_loop_i_rec(trace, loopsize)) {
            return false;
         }
      }
      trace.pop_back();
      return true;
   } else {
      /* have a full path, now need to check whether there exists another iteration */
      Node *node = trace.back();

      for (size_t i = 0; i < loopsize; ++i) {
         const auto& candidates = po.rev.at(node);
         const auto candidates_it = std::find_if(candidates.begin(), candidates.end(),
                                      [node] (Node *cmpnode) {
                                         return node->I == cmpnode->I;
                                      });
         if (candidates_it == candidates.end()) {
            /* path is dead end */
            return false;
         }

         node = *candidates_it;
      }

      return true;
   }
}

size_t AEGPO::depth(Node *dst) const {
   size_t depth_ = 1;
   for (Node *src : po.rev.at(dst)) {
      depth_ = std::max(depth_, depth(src));
   }
   return depth_;
}

void AEGPO::construct(const CFG& cfg, Node *node, MergeMap& merge_map) {
   merge_map[node->I].insert(node);
   
   const auto& succs = cfg.fwd.at(node->I);
   assert(succs.size() > 0);

   /* Branching approach:
    * At each instruction, look for nodes with same instruction in merge map. 
    * For each node, check whether it's an ancestor of the current node we're adding. 
    * If it is, then add edge to that one and stop recursing. 
    * Otherwise, proceed as normal.
    */

   /* This tracks which nodes we've created along all branches. If we find a node with a duplicate
    * instruction along another branch, we can simply merge them.
    */
   for (const llvm::Instruction *succ_I : succs) {
      const auto& merge_candidates = merge_map[succ_I];
      const auto merge_candidate_it =
         std::find_if(merge_candidates.begin(), merge_candidates.end(),
                      [this, node] (Node *merge_candidate) {
                         return !this->is_ancestor(node, merge_candidate);
                      });
      if (merge_candidate_it == merge_candidates.end()) {
         /* node not mergeable */
         Node *succ_node = new Node {succ_I};
         nodes.emplace_back(succ_node);
         add_edge(node, succ_node);
         
         /* recurse if not exit */
         if (succ_I) {
            construct(cfg, succ_node, merge_map);
         }
      } else {
         /* node mergeable */
         add_edge(node, *merge_candidate_it);
      }
   }
}



   
bool AEGPO::is_ancestor(Node *child, Node *parent) const {
   using NodeSet = std::unordered_set<Node *>;
   NodeSet ancestors {child};
   while (!ancestors.empty()) {
      if (ancestors.find(parent) != ancestors.end()) { return true; }
      NodeSet next;
      for (Node *ancestor : ancestors) {
         const auto& preds = po.rev.at(ancestor);
         next.insert(preds.begin(), preds.end());
      }
      ancestors = std::move(next);
   }
   return false;
}

void AEGPO::construct(const CFG& cfg) {
   MergeMap merge_map;
   construct(cfg, entry, merge_map);
}


llvm::raw_ostream& AEGPO::dump(llvm::raw_ostream& os) const {
#if 0
   os << "Nodes:\n";

   std::unordered_map<const Node *, unsigned> node_ids;
   size_t i = 0; 
   for (const auto& node : nodes) {
      node_ids.emplace(node.get(), i);
      os << "  " << i << "  ";
      if (node->I) {
         os << *node->I;
      } else if (node.get() == entry) {
         os << "<entry>";
      } else {
         os << "<exit>";
      }
      os << "\n";
      ++i;
   }

   os << "\n";
   os << "Edges:\n";
   for (const auto& pair : po.fwd) {
      const Node *src = pair.first;
      for (const Node *dst : pair.second) {
         os << "  " << node_ids.at(src) << " " << node_ids.at(dst) << "\n";
      }
   }

#elif 0

   // print as basic blocks
   std::unordered_map<const Node *, unsigned> node_ids;
   size_t next_id = 0;
   std::vector<Node *> stack {entry};

   while (!stack.empty()) {
      /* Get next node and ID it if necessary */
      Node *node = stack.back();
      stack.pop_back();
      const auto id_it = node_ids.find(node);
      if (id_it != node_ids.end()) {
         continue;
      }


      const Rel::Set *succ_nodes;
      while (true) {
         const auto id_it = node_ids.find(node);
         if (id_it != node_ids.end()) {
            os << "  GOTO " << id_it->second << "\n";
            goto end;
         }
         
         const auto id = next_id++;
         node_ids.emplace(node, id);
         os << id << "  ";
         node->dump(os, node == entry ? "<ENTRY>" : "<EXIT>") << "\n";
         succ_nodes = &po.fwd.at(node);
         if ((node->I && node->I == &node->I->getParent()->back()) ||
             succ_nodes->size() != 1) {
            
            break;
         }
         node = *succ_nodes->begin();
      }
      
      stack.insert(stack.end(), succ_nodes->begin(), succ_nodes->end());
      
   end:
      os << "\n";
   }


#else

   /* Approach: divide instructions into basic blocks.
    * Basic blocks have the following characteristics:
    *  - Start with an instruction that npreds != 1 or npreds == 1 and pred has != 1 successor.
    *  - Start with an instruction with any number of predecessors.
    *  - End with an instruction with any number of successors.
    *  - Middle instructions have exactly 1 pred and 1 successors.
    */

   unsigned next_bb = 0;

   std::unordered_map<Node *, unsigned> BB_starts;

   for (const auto& node_ : nodes) {
      Node *node = node_.get();
      const auto& preds = po.rev.at(node);
      if (preds.size() != 1 ||
          po.fwd.at(*preds.begin()).size() != 1) {
         /* Basic Block start */
         BB_starts.emplace(node, next_bb++);
      }
   }

   for (const auto& pair : BB_starts) {
      os << "BB " << pair.second << "\n";
      Node *node = pair.first;
      const Rel::Set *succs;

      while (true) {
         os << "  ";
         node->dump(os, node == entry ? "<ENTRY>" : "<EXIT>") << "\n";
         succs = &po.fwd.at(node);
         if (succs->size() != 1) {
            break;
         }
         node = *succs->begin();
         if (po.rev.at(node).size() != 1) {
            break;
         }
      }
         
      /* list exits */
      if (succs->size() > 0) {
         os << "    GOTO";
         for (Node *succ : *succs) {
            os << " " << BB_starts.at(succ);
         }
         os << "\n";
      }
      os << "\n";
   }
   
#endif

   return os;
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO& aeg) {
   return aeg.dump(os);
}
