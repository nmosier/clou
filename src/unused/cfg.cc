#include <cassert>

#include <llvm/IR/Instructions.h>

#include "cfg.h"

CFG::CFG(): nodes({Node {Entry {}}, Node {Exit {}}}) {
   po.add_node(entry);
   po.add_node(exit);
}

const CFG::Node& CFG::lookup(NodeRef ref) const {
   assert(ref < nodes.size());
   return nodes[ref];
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const CFG::Node& node) {
   std::visit(util::overloaded {
         [&] (CFG::Entry entry) { os << "<ENTRY>"; },
         [&] (CFG::Exit exit) { os << "<EXIT>"; },
         [&] (const llvm::Instruction *I) { os << *I; }
      }, node.v);
   return os;
}

void CFG::dump_graph(const std::string& path) const {
   po.group().dump_graph(path, [&] (auto& os, const auto& group) {
      for (const NodeRef& ref : group) {
         os << lookup(ref) << "\n";
      }
   });
}

CFG::NodeRef CFG::add_node(const llvm::Instruction *I) {
   const NodeRef ref = nodes.size();
   nodes.emplace_back(I);
   return ref;
}

void CFG::construct(const llvm::Function& F) {
   construct_rec({F, {entry}, {exit}});
   remove_calls();
}

void CFG::construct_rec(const CallSite& site) {
#if 0
   // check if call site already seen; otherwise mark as seen
   const auto seen_res = seen.insert(site);
   if (!seen_res.second) { return; }
#endif

   std::unordered_map<const llvm::Instruction *, NodeRef> refs;

   for (const llvm::BasicBlock& B : site.F) {
      for (const llvm::Instruction& I : B) {
         refs.emplace(&I, add_node(&I));
      }
   }
   
   po.insert(site.entry.begin(), site.entry.end(), refs.at(&site.F.front().front()));
   
   for (const llvm::BasicBlock& B : site.F) {
      /* add linear body of BB */
      for (auto cur = B.begin(), next = std::next(cur); next != B.end(); cur = next, ++next) {
         po.insert(refs.at(&*cur), refs.at(&*next));
      }

      /* add terminator */
      const llvm::Instruction *term = &B.back();
      const NodeRef term_ref = refs.at(term);
      const auto num_succs = term->getNumSuccessors();
      if (num_succs == 0) {
         po.insert(term_ref, site.exit.begin(), site.exit.end());
      } else {
         for (size_t i = 0; i < num_succs; ++i) {
            const llvm::Instruction *succ = &term->getSuccessor(i)->front();
            po.insert(term_ref, refs.at(succ));
         }
      }
   }

   /* process calls */
   for (const llvm::BasicBlock& B : site.F) {
      for (const llvm::Instruction& I : B) {
         if (const auto *C = llvm::dyn_cast<llvm::CallBase>(&I)) {
            const NodeRef C_ref = refs.at(C);
            const CallSite newsite {*C->getCalledFunction(), po.rev.at(C_ref), po.fwd.at(C_ref)};
            construct_rec(newsite);
         }
      }
   }
}

void CFG::remove_calls() {
   for (NodeRef ref = 0; ref < nodes.size(); ++ref) {
      if (const auto I = std::get_if<const llvm::Instruction *>(&lookup(ref).v)) {
         if (llvm::dyn_cast<llvm::CallBase>(*I)) {
            po.erase(ref);
         }
      }
      
   }
}
