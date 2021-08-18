
#include "aeg-po2.h"
#include "util.h"

/* Construction Algorithm
 * We will construct functions recursively. Constructing a function should return the
 * entry and exit nodes.
 * While constructing each function, we will then proceed with the regular flow. 
 *
 */


llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO_Node_Base& node) {
   std::visit(util::overloaded {
         [&] (Entry) { os << "<ENTRY>"; },
         [&] (Exit)  { os << "<EXIT>";  },
         [&] (const llvm::Instruction *I) { os << *I; },
      }, node());
   if (node.func_id) {
      os << " F" << *node.func_id;
   }
   if (!node.loop_id.empty()) {
      os << " L";
      for (auto loop_id : node.loop_id) {
         os << loop_id << ",";
      }
   }
   return os;
}

void AEGPO2::prune() {
   std::unordered_set<NodeRef> todo;
   for (NodeRef i = 0; i < nodes.size(); ++i) {
      if (i != exit) {
         todo.insert(i);
      }
   }

   std::unordered_set<NodeRef> deleted;
   while (!todo.empty()) {
      // pop off next job
      const auto it = todo.begin();
      const NodeRef ref = *it;
      todo.erase(it);

      if (po.fwd.at(ref).empty()) {
         // is leaf
         const auto& preds = po.rev.at(ref);
         std::copy(preds.begin(), preds.end(), std::inserter(todo, todo.end()));
         po.erase(ref);
         deleted.insert(ref);
      }
   }

   /* NOTE: There is a more efficient way to do this renumbering of node references; 
    * however, this method preserved the order. If it becomes too slow, we can replace it.
    */

   /* create mapping from old ref to new refs */
   std::unordered_map<NodeRef, NodeRef> refmap;
   NodeRef next_ref = 0;
   for (NodeRef old_ref = 0; old_ref < nodes.size(); ++old_ref) {
      if (deleted.find(old_ref) == deleted.end()) {
         refmap.emplace(old_ref, next_ref);
         ++next_ref;
      }
   }

   /* compactify nodes */
   Rel new_po;
   for (NodeRef old_ref = 0; old_ref < nodes.size(); ++old_ref) {
      const auto it = refmap.find(old_ref);
      if (it != refmap.end()) {
         nodes[it->second] = std::move(nodes[it->first]);
         new_po.add_node(it->second);
      }
   }
   nodes.resize(next_ref);
   
   /* rename mappings */
   for (const auto& pair : po.fwd) {
      const NodeRef src = pair.first;
      for (const NodeRef dst : pair.second) {
         new_po.insert(refmap.at(src), refmap.at(dst));
      }
   }
   po = std::move(new_po);
}

bool AEGPO2::alias_valid(const Node& a, const Node& b) {
   if (!(a.func_id && b.func_id)) {
      return false;
   }
   if (*a.func_id != *b.func_id) {
      return false;
   }

   if (!std::equal(a.loop_id.begin(),
                   a.loop_id.begin() + std::min(a.loop_id.size(), b.loop_id.size()),
                   b.loop_id.begin())) {
      return false;
   }

   return true;
}
