
#include "cfg/cfg.h"
#include "util.h"

/* Construction Algorithm
 * We will construct functions recursively. Constructing a function should return the
 * entry and exit nodes.
 * While constructing each function, we will then proceed with the regular flow. 
 *
 */


llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const CFG::Node& node) {
   std::visit(util::overloaded {
         [&] (Entry) { os << "<ENTRY>"; },
         [&] (Exit)  { os << "<EXIT>";  },
         [&] (const llvm::Instruction *I) { os << *I; },
       [&] (const CFG::Node::Call& call) {
           os << *call.C << " " << call.arg;
       },
      }, node());
   if (node.id) {
      os << " F" << node.id->func;
      os << " L";
      for (auto loop_id : node.id->loop) {
         os << loop_id << ",";
      }
   }
   return os;
}

void CFG::prune() {
   std::unordered_set<NodeRef> todo;
   for (NodeRef i = 0; i < nodes.size(); ++i) {
       if (exits.find(i) == exits.end()) {
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

bool CFG::alias_valid(const ID& a, const ID& b) {
   if (a.func != b.func) {
      return false;
   }

   if (!std::equal(a.loop.begin(), a.loop.begin() + std::min(a.loop.size(), b.loop.size()),
                   b.loop.begin())) {
      return false;
   }

   return true;
}

bool CFG::alias_valid(const Node& a, const Node& b) {
   if (!(a.id && b.id)) {
      return false;
   }
   return alias_valid(*a.id, *b.id);
}

bool CFG::postorder_rec(NodeRefSet& done, NodeRefVec& order, NodeRef ref) const {
   if (done.find(ref) != done.end()) {
      return true;
   }

   bool acc = true;
   for (NodeRef succ : po.fwd.at(ref)) {
      acc &= postorder_rec(done, order, succ);
   }

   if (acc) {
      done.insert(ref);
      order.push_back(ref);
   }

   return acc;
}

NodeRef CFG::add_node(const Node& node) {
      const NodeRef ref = size();
      nodes.push_back(node);
      po.add_node(ref);
      return ref;
   }   

std::ostream& operator<<(std::ostream& os, const CFG::ID& id) {
    os << "F" << id.func << " L{";
    for (auto it = id.loop.begin(); it != id.loop.end(); ++it) {
        if (it != id.loop.begin()) {
            os << " ";
        }
        os << *it;
    }
    os << "}";
    return os;
}

bool CFG::is_block_boundary(NodeRef ref, const Rel::Map& fwd, const Rel::Map& rev) const {
    const auto& preds = rev.at(ref);
    if (preds.size() != 1) {
        return true;
    }
    const NodeRef pred = *preds.begin();
    const auto& pred_succs = fwd.at(pred);
    if (pred_succs.size() == 1) {
        return false;
    } else {
        return true;
    }
}

bool CFG::is_block_entry(NodeRef ref) const {
    return is_block_boundary(ref, po.fwd, po.rev);
}

bool CFG::is_block_exit(NodeRef ref) const {
    return is_block_boundary(ref, po.rev, po.fwd);
}

std::optional<NodeRef> CFG::get_block_successor(NodeRef ref) const {
    if (is_block_exit(ref)) {
        return std::nullopt;
    } else {
        return *po.fwd.at(ref).begin();
    }
}

std::ostream& operator<<(std::ostream& os, const CFG::Node::Call& call) {
    std::string s;
    llvm::raw_string_ostream ss {s};
    ss << *call.C << " " << *call.arg;
    return os << ss.str();
}

std::ostream& operator<<(std::ostream& os, const CFG::Node::Variant& v) {
    std::visit(util::overloaded {
        [&] (const auto *x) {
            std::string s;
            llvm::raw_string_ostream ss {s};
            ss << *x;
            os << ss.str();
        },
        [&] (const auto& x) {
            os << x;
        },
    }, v);
    return os;
}

bool CFG::Node::may_read() const {
    return std::visit(util::overloaded {
        [&] (Entry) { return false; },
        [&] (Exit) { return true; },
        [&] (const llvm::Instruction *I) {
            // TODO: more cases
            if (llvm::isa<llvm::LoadInst>(I)) {
                return true;
            } else {
                return false;
            }
        },
        [&] (const Call& call) {
            assert(call.arg->getType()->isPointerTy());
            return true;
        },
    }, v);
}
