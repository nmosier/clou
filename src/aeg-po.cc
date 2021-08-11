#include <cstdlib>
#include <algorithm>
#include <cassert>
#include <deque>

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

#if 1
void AEGPO::Node::dump(llvm::raw_ostream& os, const CFG& cfg) const {
   os << cfg.lookup(cfg_ref);
}
#endif


void AEGPO::add_edge(NodeRef src, NodeRef dst) {
   /* anything that passes into source and out of dst is now transitively connected. */
   po.insert(src, dst);
}


template <typename OutputIt>
OutputIt AEGPO::construct2_rec(unsigned num_unrolls,
                               MergeMap& merge_map, ConstructionTask& task, OutputIt out) {
   const NodeRef node = task.node;
   task.trace.push_back(node);

   const auto& succs = cfg.po.fwd.at(lookup(node).cfg_ref);
   assert(succs.size() > 0);

   for (const CFG::NodeRef succ_I : succs) {
      const auto& merge_candidates = merge_map[succ_I];
      const std::optional<NodeRef> merge_candidate = is_any_not_ancestor(node, merge_candidates);
      // const bool mergable = static_cast<bool>(merge_candidate);
#if 0
      // TODO: Parallelize this.
      const auto merge_candidate_it =
      std::find_if(merge_candidates.begin(), merge_candidates.end(),
                   [&] (Node *merge_candidate) {
                      return is_mergable(node, merge_candidate, trace);
                   });
      assert(mergable == merge_candidate_it != merge_candidates.end());
#endif

#if 0
      if (verbose > 0) { 
         llvm::errs() << (mergable ? "mergable" : "not mergable") << " ";
         llvm::errs() << node_id(node) << " {";
         for (Node *candidate : merge_candidates) {
            llvm::errs() << " " << node_id(candidate);
         }
         llvm::errs() << "}\n";
      }
#endif

      /* check for loops */
      auto& count = task.reps[succ_I];
      const auto oldcount = count;
      if (count == num_unrolls + 1) {
#if 0
         if (verbose > 0) {
            llvm::errs() << "aborting loop at " << *succ_I << "\n";
         }
#endif
         continue;
      }
      ++count;
      if (count >= 2) {
         // we just doubled back on a previous instruction, so clear the counts of all intervening
         // instructions
         const auto first = std::find_if(task.trace.rbegin(), task.trace.rend(),
                                         [&] (NodeRef node) {
                                            return lookup(node).cfg_ref == succ_I;
                                         });
         assert(first != task.trace.rend());
         for (auto it = task.trace.rbegin(); it != first; ++it) {
            
            task.reps[lookup(*it).cfg_ref] = 0;
         }
      }

      NodeRef succ_node;
      if (merge_candidate) {
         succ_node = *merge_candidate;
      } else {
         const auto& po_succs = po.fwd.at(node);
         const auto po_succ_it = std::find_if(po_succs.begin(), po_succs.end(),
                                              [&] (NodeRef succ) {
                                                 return succ_I == lookup(succ).cfg_ref;
                                              });
         if (po_succ_it == po_succs.end()) {
            succ_node = add_node(succ_I);
         } else {
            succ_node = *po_succ_it;
         }
      }
      add_edge(node, succ_node);
      
      /* recurse if not exit */
      merge_map[lookup(succ_node).cfg_ref].insert(succ_node);
      if (!cfg.is_exit(succ_I)) {
         *out++ = ConstructionTask {succ_node, task.reps, task.trace};
      }

      count = oldcount;
   }

   return out;
}


void AEGPO::construct2(unsigned num_unrolls) {
   MergeMap merge_map;
   std::deque<ConstructionTask> queue {
      ConstructionTask {entry, RepMap {}, NodeVec {}}
   };

   while (!queue.empty()) {
      ConstructionTask& task = queue.front();
      construct2_rec(num_unrolls, merge_map, task, std::back_inserter(queue));
      queue.pop_front();
   }

   prune();

#if 0
   // check if exits
   std::unordered_set<NodeRef> exits;
   for (const Node& node : nodes) {
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
#endif
}
   
bool AEGPO::is_ancestor_a(NodeRef child, NodeRef parent) const {
   // base case 
   if (child == parent) {
      return true;
   }

   // recursive case
   const auto& preds = po.rev.at(child);
   return std::any_of(preds.begin(), preds.end(),
                      [=] (NodeRef node) {
                         return is_ancestor_a(node, parent);
                      });
}

bool AEGPO::is_ancestor_d(NodeRef child, NodeRef parent) const {
   const Rel::Set init_set = {child};
   if (child == parent) { return true; }
   std::vector<const Rel::Set *> todo {&init_set};
   static cache_set<NodeRef, 0x1000> seen;
   seen.clear();
   while (!todo.empty()) {
      const Rel::Set& nodes = *todo.back();
      todo.pop_back();
      for (NodeRef node : nodes) {
         if (node == parent) {
            return true;
         }
         if (seen.contains(node) == seen.YES) { continue; }
         seen.insert(node);
         todo.push_back(&po.rev.at(node));
      }
   }
   return false;
}

std::optional<AEGPO::NodeRef> AEGPO::is_any_not_ancestor(NodeRef child, Rel::Set parents) const {
   const Rel::Set init_set = {child};
   std::vector<const Rel::Set *> todo {&init_set};
   static cache_set<NodeRef, 0x1000> seen;
   seen.clear();

   while (true) {
      if (parents.empty()) { return std::nullopt; }
      if (todo.empty()) { return *parents.begin(); }
      
      const Rel::Set& nodes = *todo.back();
      todo.pop_back();
      for (NodeRef node : nodes) {
         parents.erase(node);
         if (seen.contains(node) == seen.YES) { continue; }
         seen.insert(node);
         todo.push_back(&po.rev.at(node));
      }
   }
}

#if 0
bool AEGPO::is_ancestor_e(NodeRef child, NodeRef parent) const {
   std::unordered_set<Node *> seen;
   const Rel::Set init_child_set = {child};
   const Rel::Set init_parent_set = {parent};
   std::vector<std::pair<bool, const Rel::Set *>> todo = {{true, &init_child_set},
                                                          {false, &init_parent_set}};
   while (!todo.empty()) {
      const auto& pair = todo.back();
      todo.pop_back();
      const auto& rel = pair.first ? po.rev : po.fwd;
      for (Node *node : *pair.second) {
         if (!seen.insert(node).second) {
            return true;
         }
         todo.emplace_back(pair.first, &rel.at(node));
      }
   }
   
   return false;
}
#endif

#if 0
bool AEGPO::is_ancestor_b(Node *child, Node *parent) const {
   const auto it = po_trans.fwd.find(parent);
   if (it == po_trans.fwd.end()) {
      return false;
   }
   return it->second.find(child) != it->second.end();
}
#endif

bool AEGPO::is_ancestor(NodeRef child, NodeRef parent) const {
   // const bool a = is_ancestor_a(child, parent);
   // const bool b = is_ancestor_b(child, parent);
   const bool d = is_ancestor_d(child, parent);
   // const bool e = is_ancestor_e(child, parent);
   // assert(a == d);
   return d;
}

#if 0
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
#endif

void AEGPO::dump_graph(const std::string& path) const {
   po.group().dump_graph(path, [&] (auto& os, const auto& group) {
      for (const NodeRef& ref : group) {
         lookup(ref).dump(os, cfg);
         os << "\n";
      }
   });
   
   //   const auto bbrel = get_bbs(); 
   // bbrel.dump_graph(path, BBPrinter {*this});
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO& aeg) {
   return aeg.dump(os);
}

#if 0
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
#endif

void AEGPO::prune() {
   // TODO: A faster way would be to find the set of leaves and then check leaves' parents.
   NodeRef ref {0};
   std::unordered_set<NodeRef> removed;

   for (NodeRef ref {0}; ref != NodeRef {static_cast<unsigned>(nodes.size())}; ) {
      if (removed.find(ref) == removed.end() && !is_exit(ref) && po.fwd.at(ref).empty()) {
         erase(ref);
         removed.insert(ref);
         ref = NodeRef {0};
      } else {
         ++ref;
      }
   }
}

void AEGPO::erase(NodeRef node) {
   po.erase(node);
}

#if 0
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
#endif

#if 0
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
#endif

bool AEGPO::is_mergable(NodeRef node, NodeRef merge_candidate, const NodeVec& trace) const {
   // look in execution trace (heuristic)
#if 1
   if (std::find(trace.rbegin(), trace.rend(), merge_candidate) != trace.rend()) {
      return false;
   }
#endif

   // slow fallback: is_ancestor_a
   return !is_ancestor(node, merge_candidate);
}
