#include <cstdlib>
#include <algorithm>
#include <cassert>
#include <list>

#include <llvm/IR/Instructions.h>

#include "aeg-po.h"
#include "cache.h"
#include "config.h"

/* Node Merging
 * Note that there should be a total ordering among all merge candidates.
 * But this assumption might be easy to break later on. Might not even hold now.
 *
 *
 * Heuristics for merging.
 * - Check whether the merge candidate is in the execution trace. If it is, it isn't mergable.
 */


void AEGPO::add_edge(Node *src, Node *dst) {
   /* anything that passes into source and out of dst is now transitively connected. */
   po.insert(src, dst);
#if 0
   po_trans.insert(src, src);
   po_trans.insert(dst, dst);
   const auto srcs = po_trans.rev[src];
   const auto dsts = po_trans.fwd[dst];
   po_trans.insert(srcs.begin(), srcs.end(), dsts.begin(), dsts.end());
   assert(po_trans.fwd.at(src).find(dst) != po_trans.fwd.at(src).end());
#endif
#if 0
   add_children(src, dst);
#endif

   // update depths
   
}

void AEGPO::add_children(Node *src, Node *dst) {
   std::unordered_set<Node *> newchildren = po_children.fwd[dst];
   newchildren.insert(dst);
   std::vector<Node *> nodes {src};
   while (!nodes.empty()) {
      Node *node = nodes.back();
      nodes.pop_back();
      po_children.insert(node, newchildren.begin(), newchildren.end());
      const auto& preds = po.rev.at(node);
      std::copy(preds.begin(), preds.end(), std::back_inserter(nodes));
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

template <typename OutputIt>
void AEGPO::construct2_rec(const CFG2& cfg, unsigned num_unrolls, Node *node, MergeMap& merge_map,
                           RepMap reps, NodeVec trace, OutputIt& out) {
   trace.push_back(node);

   const auto& succs = cfg.po.fwd.at(node->I);
   assert(succs.size() > 0);

   for (const llvm::Instruction *succ_I : succs) {
      const auto& merge_candidates = merge_map[succ_I];
      const auto merge_candidate_it =
         // TODO: Parallelize this.
         std::find_if(merge_candidates.begin(), merge_candidates.end(),
                      [&] (Node *merge_candidate) {
                         return is_mergable(node, merge_candidate, trace);
                      });
      const bool mergable = merge_candidate_it != merge_candidates.end();

      if (verbose > 0) { 
         llvm::errs() << (mergable ? "mergable" : "not mergable") << " ";
         llvm::errs() << node_id(node) << " {";
         for (Node *candidate : merge_candidates) {
            llvm::errs() << " " << node_id(candidate);
         }
         llvm::errs() << "}\n";
      }

      /* check for loops */
      auto& count = reps[succ_I];
      const auto oldcount = count;
      if (count == num_unrolls + 1) {
         if (verbose > 0) {
            llvm::errs() << "aborting loop at " << *succ_I << "\n";
         }
         continue;
      }
      ++count;
      if (count >= 2) {
         // we just doubled back on a previous instruction, so clear the counts of all intervening
         // instructions
         const auto f = [succ_I] (const Node *node) { return node->I == succ_I; };
         const auto first = std::find_if(trace.rbegin(), trace.rend(), f);
         assert(first != trace.rend());
         // Loop loop = {(**first).I}; // also include first instruction 
         for (auto it = trace.rbegin(); it != first; ++it) {
            const llvm::Instruction *I = (**it).I;
            reps[I] = 0;
            // loop.insert(I);
         }
         // loops.insert(std::move(loop));
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
            po.add_node(succ_node);
         } else {
            succ_node = *po_succ_it;
         }
      }
      add_edge(node, succ_node);
      
      /* recurse if not exit */
      merge_map[succ_node->I].insert(succ_node);
      if (succ_I) {
         *out++ = [&, num_unrolls, succ_node, reps, trace] {
            construct2_rec(cfg, num_unrolls, succ_node, merge_map, reps, trace, out); 
         };
      }

      count = oldcount;
   }
}


void AEGPO::construct2(const CFG2& cfg, unsigned num_unrolls) {
   MergeMap merge_map;
   RepMap reps;
   NodeVec trace;
   std::list<std::function<void(void)>> queue;
   auto out = std::back_inserter(queue);
   *out++ = [&, reps, trace, num_unrolls] () {
      construct2_rec(cfg, num_unrolls, entry, merge_map, reps, trace, out);
   };

   while (!queue.empty()) {
      queue.front()();
      queue.pop_front();
   }

   prune();

   // check if exits
   std::unordered_set<Node *> exits;
   for (const auto& nodeptr : nodes) {
      Node *node = nodeptr.get();
      if (is_exit(node)) {
         exits.insert(node);
      }
   }
   llvm::errs() << "EXITS\n";
   for (auto it1 = exits.begin(); it1 != exits.end(); ++it1) {
      for (auto it2 = it1; it2 != exits.end(); ++it2) {
         if (is_ancestor(*it1, *it2)) {
            llvm::errs() << node_id(*it1) << " " << node_id(*it2) << "\n";
         }
      }
   }
}
   
bool AEGPO::is_ancestor_a(Node *child, Node *parent) const {
   // base case 
   if (child == parent) {
      return true;
   }

   // recursive case
   const auto& preds = po.rev.at(child);
   return std::any_of(preds.begin(), preds.end(),
                      [=] (Node *node) {
                         return is_ancestor_a(node, parent);
                      });
}

bool AEGPO::is_ancestor_d(Node *child, Node *parent) const {
   const Rel::Set init_set = {child};
   if (child == parent) { return true; }
   std::vector<const Rel::Set *> todo {&init_set};
   static cache_set<Node *, 0x1000> seen;
   seen.clear();
#if 0
   while (!todo.empty()) {
      for (Node *node : todo) {
         if (node == parent) {
            return true;
         }
         if (seen.contains(node) == seen.YES) {
            continue;
         }
         seen.insert(node);
         const auto& preds = po.rev.at(node);
         std::copy(preds.begin(), preds.end(), std::back_inserter(todo_next));
      }
      todo = std::move(todo_next);
   }
#elif 0
   while (!todo.empty()) {
      Node *node = todo.back();
      todo.pop_back();
      if (node == parent) { return true; }
      if (seen.contains(node) == seen.YES) { continue; }
      seen.insert(node);
      const auto& preds = po.rev.at(node);
      std::copy(preds.begin(), preds.end(), std::back_inserter(todo));
   }
#else
   while (!todo.empty()) {
      const Rel::Set& nodes = *todo.back();
      todo.pop_back();
      for (Node *node : nodes) {
         if (node == parent) { return true; }
         if (seen.contains(node) == seen.YES) { continue; }
         seen.insert(node);
         todo.push_back(&po.rev.at(node));
      }
   }
#endif
   return false;
}

bool AEGPO::is_ancestor_b(Node *child, Node *parent) const {
   const auto it = po_trans.fwd.find(parent);
   if (it == po_trans.fwd.end()) {
      return false;
   }
   return it->second.find(child) != it->second.end();
}

bool AEGPO::is_ancestor(Node *child, Node *parent) const {
   // const bool a = is_ancestor_a(child, parent);
   // const bool b = is_ancestor_b(child, parent);
   const bool d = is_ancestor_d(child, parent);
   // assert(a == d);
   return d;
#if 0
   if (a != b) {
      // dump po, po_trans
      po.dump_graph("po.dot", [] (llvm::raw_ostream& os, const Node *node) {
         node->dump(os, "<ENTRY/EXIT>") << "\n";
      });
      po_children.dump_graph("po_trans.dot", [] (llvm::raw_ostream& os, const Node *node) {
         node->dump(os, "<ENTRY/EXIT>") << "\n";
      });
   }
   
   assert(a == b);
   return a;
#endif
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
   
   return os;
}

void AEGPO::dump_graph(const std::string& path) const {
   struct NodePrinter {
      const AEGPO& aeg;
      
      NodePrinter(const AEGPO& aeg): aeg(aeg) {}
      
      void operator()(llvm::raw_ostream& os, const Node *node) const {
#if 0
         os << std::find_if(aeg.nodes.begin(), aeg.nodes.end(),
                            [node] (const auto& ref) {
                               return ref.get() == node;
                            }) - aeg.nodes.begin() << "\n";
#endif
         node->dump(os, node == aeg.entry ? "<ENTRY>" : "<EXIT>") << "\n";
      }
   };

   struct BBPrinter {
      NodePrinter node_printer;
      BBPrinter(const AEGPO& aeg): node_printer(aeg) {}
      void operator()(llvm::raw_ostream& os, const BB& bb) const {
         for (const Node *node : bb) {
            node_printer(os, node); 
         }
      }
   };

   const auto bbrel = get_bbs(); 
   bbrel.dump_graph(path, BBPrinter {*this});
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO& aeg) {
   return aeg.dump(os);
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

void AEGPO::prune() {
   // TODO: A faster way would be to find the set of leaves and then check leaves' parents.
   for (auto it = nodes.begin(); it != nodes.end(); ) {
      Node *node = it->get();
      if (node->I && po.fwd.at(node).empty()) {
            erase(node);
            it = nodes.begin();
      } else {
         ++it;
      }
   }
}

void AEGPO::erase(Node *node) {
   po.erase(node);
   llvm::erase_if(nodes, [node] (const auto& nodep) {
      return node == nodep.get();
   });
}

binrel<AEGPO::BB> AEGPO::get_bbs() const {
   binrel<BB> rel;

   for (const auto& nodeptr : nodes) {
      Node *node = nodeptr.get();
      rel.add_node(get_bb(node));
   }

   for (const auto& nodeptr : nodes) {
      Node *src = nodeptr.get();
      const BB src_bb = get_bb(src);
      for (Node *dst : po.fwd.at(src)) {
         const BB dst_bb = get_bb(dst);
         if (src_bb != dst_bb) {
            rel.insert(src_bb, dst_bb);
         }
      }
   }

   return rel;
}


AEGPO::BB AEGPO::get_bb(Node *node) const {
   BB bb;

   /* find entry */
   while (true) {
      const auto& preds = po.rev.at(node);
      if (preds.size() != 1) { break; }
      Node *pred = *preds.begin();
      if (po.fwd.at(pred).size() != 1) { break; }
      node = pred;
   }

   /* construct BB */
   while (true) {
      bb.push_back(node);
      const auto& succs = po.fwd.at(node);
      if (succs.size() != 1) { break; }
      Node *succ = *succs.begin();
      if (po.rev.at(succ).size() != 1) { break; }
      node = succ;
   }

   return bb;
}


bool AEGPO::is_exit(Node *node) const {
   return node->I == nullptr && po.fwd.at(node).empty();
}


unsigned AEGPO::depth(Node *node) const {
   const auto& preds = po.rev.at(node);
   if (preds.empty()) {
      return 0;
   }
   std::vector<unsigned> subdepths(preds.size());
   std::transform(preds.begin(), preds.end(), subdepths.begin(),
                  [&] (Node *pred) {
                     return depth(pred);
                  });
   return *std::max_element(subdepths.begin(), subdepths.end());
}

AEGPO::Node *AEGPO::nearest_common_ancestor(Node *a, Node *b) const {
   // TODO
   std::abort();
}

bool AEGPO::is_mergable(Node *node, Node *merge_candidate, const NodeVec& trace) const {
   // look in execution trace (heuristic)
#if 0
   if (std::find(trace.rbegin(), trace.rend(), merge_candidate) != trace.rend()) {
      return false;
   }
#endif

   // slow fallback: is_ancestor_a
   return !is_ancestor(node, merge_candidate);
}
