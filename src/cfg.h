#pragma once

#include <string>
#include <variant>

#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>

#include "binrel.h"
#include "noderef.h"
#include "util.h"

class CFG {
public:
   using NodeRef = ::NodeRef<CFG>;
   struct Entry {};
   struct Exit {};

   struct Node {
      using Variant = std::variant<Entry, Exit, const llvm::Instruction *>;
      Variant v;
      template <typename... Ts>
      Node(Ts&&... ts): v(std::forward<Ts>(ts)...) {}

      bool is_exit() const { return std::holds_alternative<Exit>(v); }
      friend llvm::raw_ostream& operator<<(llvm::raw_ostream&, const Node&);
   };

   using Rel = binrel<NodeRef>;
   Rel po;

   CFG();

   static inline const NodeRef entry {0};
   static inline const NodeRef exit  {1};
   
   void construct(const llvm::Function& F);

   const Node& lookup(NodeRef ref) const;
   
   struct CallSite {
      using Set = std::unordered_set<NodeRef>;
      const llvm::Function& F;
      Set entry;
      Set exit;
      friend struct std::hash<CallSite>;
      bool operator==(const CallSite& other) const {
         return &F == &other.F && entry == other.entry && exit == other.exit;
      }
   };

   void dump_graph(const std::string& path) const;

   bool is_exit(NodeRef node) const { return lookup(node).is_exit(); }
   
private:
   std::vector<Node> nodes;
   
   using CallSites = std::unordered_set<CallSite>;
   void construct(const CallSite& site, CallSites& sites);
   void remove_calls();

   NodeRef add_node(const llvm::Instruction *I);
};

namespace std {
   template <> 
   struct hash<CFG::CallSite> {
      size_t operator()(const CFG::CallSite& site) const {
         return hash_ordered_tuple(&site.F, site.entry, site.exit);
      }
   };
}
