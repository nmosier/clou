#include <cstdlib>
#include <algorithm>
#include <cassert>

#include <llvm/IR/Instructions.h>

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

#if 0
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
#endif

size_t AEGPO::depth(Node *dst) const {
   size_t depth_ = 1;
   for (Node *src : po.rev.at(dst)) {
      depth_ = std::max(depth_, depth(src));
   }
   return depth_;
}

void AEGPO::construct(const CFG& cfg, Node *node, MergeMap& merge_map, const RepMap& reps_,
                      NodeVec& trace) {
   merge_map[node->I].insert(node);
   trace.push_back(node);
   
   const auto& succs = cfg.fwd.at(node->I);
   assert(succs.size() > 0);

   for (const llvm::Instruction *succ_I : succs) {
      RepMap reps = reps_;
      
      const auto& merge_candidates = merge_map[succ_I];
      RepMap node_reps_tmp = {{succ_I, 1}};
      const unsigned node_max_reps = max_reps(node, node_reps_tmp);
      const auto merge_candidate_it =
         std::find_if(merge_candidates.begin(), merge_candidates.end(),
                      [this, node, node_max_reps] (Node *merge_candidate) {
                         return !this->is_ancestor(node, merge_candidate) &&
                            (node_max_reps == max_reps(merge_candidate) || true);
                      });
      const bool mergable = merge_candidate_it != merge_candidates.end();
      
      llvm::errs() << (mergable ? "mergable" : "not mergable") << " ";
      if (succ_I) {
         llvm::errs() << *succ_I;
      } else {
         llvm::errs() << "<EXIT>";
      }
      llvm::errs() << "     " << merge_candidates.size();
      llvm::errs() << "\n";

      /* check for loops */
      auto& count = reps[succ_I];
      if (count == 2) {
         llvm::errs() << "aborting loop at " << *succ_I << "\n";
         continue;
      }
      ++count;
      if (count == 2) {
         // we just doubled back on a previous instruction, so clear the counts of all intervening
         // instructions
         const auto f = [succ_I] (const Node *node) { return node->I == succ_I; };
         const auto first = std::find_if(trace.rbegin(), trace.rend(), f);
         assert(first != trace.rend());
         for (auto it = trace.rbegin(); it != first; ++it) {
            llvm::errs() << "resetting " << *(**it).I << "\n";
            reps[(**it).I] = 0;
         }
      }
      
      Node *succ_node;
      if (mergable) {
         succ_node = *merge_candidate_it;
      } else {
         const auto& po_succs = po.fwd.at(node);
         const auto po_succ_it = std::find_if(po_succs.begin(), po_succs.end(),
                                              [succ_I] (const Node *succ) {
                                                 return succ_I == succ->I;
                                              });
         if (po_succ_it == po_succs.end()) {
            succ_node = new Node {succ_I};
            nodes.emplace_back(succ_node);
         } else {
            succ_node = *po_succ_it;
         }
      }
      add_edge(node, succ_node);      
      
      /* recurse if not exit */
      if (succ_I) {
         construct(cfg, succ_node, merge_map, reps, trace);
      }
   }

   trace.pop_back();
}



   
bool AEGPO::is_ancestor(Node *child, Node *parent) const {
#if 0
   const auto it = po_trans.rev.find(child);
   if (it == po_trans.rev.end()) {
      return false;
   } else {
      return it->second.find(parent) != it->second.end(); 
   }
#elif 0
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
#else
   // base case 
   if (child == parent) {
      return true;
   }

   // recursive case
   const auto& preds = po.rev.at(child);
   return std::any_of(preds.begin(), preds.end(),
                      [parent, this] (Node *node) {
                         return this->is_ancestor(node, parent);
                      });
#endif
}

void AEGPO::construct(const CFG& cfg) {
   MergeMap merge_map;
   RepMap reps;
   NodeVec trace;
   construct(cfg, entry, merge_map, reps, trace);

   const auto switches = std::count_if(nodes.begin(), nodes.end(), [] (const auto& node) {
      return llvm::dyn_cast_or_null<llvm::SwitchInst>(node->I) != nullptr;
   });
   llvm::errs() << "switches " << switches << "\n";

   const auto it = std::find_if(merge_map.begin(), merge_map.end(), [] (const auto& pair) {
      return llvm::dyn_cast_or_null<llvm::SwitchInst>(pair.first) != nullptr;
   });
   if (it != merge_map.end()) {
      const auto& set = it->second;
      for (auto it1 = set.begin(); it1 != set.end(); ++it1) {
         for (auto it2 = std::next(it1); it2 != set.end(); ++it2) {
            if (!is_ancestor(*it1, *it2)) {
               (*it1)->dump(llvm::errs(), "<ENTRY>") << "\n";
            }
         }
      }
   }
   
}


llvm::raw_ostream& AEGPO::dump(llvm::raw_ostream& os) const {
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


   // DEBUG: show mapping and graph
   struct Printer {
      const AEGPO& aeg;
      
      Printer(const AEGPO& aeg): aeg(aeg) {}
      
      void operator()(llvm::raw_ostream& os, const Node *node) const {
         os << std::find_if(aeg.nodes.begin(), aeg.nodes.end(),
                            [node] (const auto& ref) {
                               return ref.get() == node;
                            }) - aeg.nodes.begin() << "\n";
         node->dump(os, "<ENTRY/EXIT>");
      }
   };
   po.dump_graph("po.dot", Printer {*this});
   
   return os;
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO& aeg) {
   return aeg.dump(os);
}


bool AEGPO::is_sibling(Node *a, Node *b) const {
   return !is_ancestor(a, b) && !is_ancestor(b, a);
}

unsigned AEGPO::max_reps(Node *node, RepMap reps) const {
   ++reps[node->I];
   const auto& preds = po.rev.at(node);
   
   if (preds.size() == 0) {
      /* base case */
      assert(node == entry);
      const auto it = std::max_element(reps.begin(), reps.end(),
                                       [] (const auto& a, const auto& b) {
                                          return a.second < b.second;
                                       });
      return it == reps.end() ? 0 : it->second;
   }

   std::vector<unsigned> maxes;
   maxes.resize(preds.size());
   std::transform(preds.begin(), preds.end(), maxes.begin(),
                  [this, &reps] (Node *pred) {
                     return this->max_reps(pred, reps);
                  });
   return *std::max_element(maxes.begin(), maxes.end());
}

