#include <llvm/IR/Dominators.h>

#include "aeg-po2.h"
#include "util.h"

/* Construction Algorithm
 * We will construct functions recursively. Constructing a function should return the
 * entry and exit nodes.
 * While constructing each function, we will then proceed with the regular flow. 
 *
 */

void AEGPO2::construct() {
   entry = add_node(Node::make_entry());
   exit = add_node(Node::make_exit());
   Port port;
   construct_function(&F, port);
   add_edge(entry, port.entry);
   for (const auto& exit_pair : port.exits) {
      add_edge(exit_pair.second, exit);
   }
   prune();
}

template <typename OutputIt>
void AEGPO2::construct_instruction(const llvm::Instruction *I, Port& port, OutputIt out) {
   if (const llvm::CallBase *C = llvm::dyn_cast<llvm::CallBase>(I)) {
      construct_call(C, port, out); 
   } else {
      port.entry = add_node(Node::make(I));
      port.exits = {{I->getParent(), port.entry}};
      *out++ = port.entry;
   }
}

template <typename OutputIt>
void AEGPO2::construct_call(const llvm::CallBase *C, Port& port, OutputIt out) {
   construct_function(C->getCalledFunction(), port);
}

template <typename OutputIt>
void AEGPO2::construct_block(const llvm::BasicBlock *B, Port& port, OutputIt out) {
   std::vector<Port> ports;
   ports.resize(B->size());

   auto port_it = ports.begin();
   for (const llvm::Instruction& I : *B) {
      construct_instruction(&I, *port_it, out);
      if (port_it != ports.begin()) {
         auto port_prev = std::next(port_it, -1);
         connect(port_prev->exits.begin(), port_prev->exits.end(), port_it->entry);
      }
      ++port_it;
   }
   
   port.entry = ports.front().entry;
   port.exits = std::move(ports.back().exits);
}

template <typename OutputIt>
void AEGPO2::construct_loop_forest(const LoopForest *LF, Port& port, OutputIt out) {
   /* construct loops */
   std::unordered_map<const llvm::BasicBlock *, const llvm::Loop *> block_to_loop;
   std::unordered_map<const llvm::Loop *, Port> loop_to_port;
   for (const llvm::Loop *subL : LF->loops) {
      /* process loop */
      Port subL_port;
      construct_loop(subL, subL_port);
      for (const llvm::BasicBlock *B : subL->blocks()) {
         block_to_loop.emplace(B, subL);
      }
      loop_to_port.emplace(subL, subL_port);
   }

   /* construct non-loop basic blocks */
   std::unordered_map<const llvm::BasicBlock *, Port> nonloop_block_to_port;
   for (const llvm::BasicBlock *B : LF->blocks) {
      if (block_to_loop.find(B) == block_to_loop.end()) {
         Port B_port;
         construct_block(B, B_port, out);
         nonloop_block_to_port.emplace(B, B_port);
      }
   }

   /* connect ports */
   const auto get_block_port = [&] (const llvm::BasicBlock *succ, const llvm::Loop *L) -> Port * {
      const auto succ_loop_it = block_to_loop.find(succ);
      if (succ_loop_it != block_to_loop.end()) {
         if (succ_loop_it->second == L) {
            return nullptr; // would be within same loop
         } else {
            return &loop_to_port.at(succ_loop_it->second);
         }
      } else {
         const auto nonloop_it = nonloop_block_to_port.find(succ);
         if (nonloop_it != nonloop_block_to_port.end()) {
            return &nonloop_it->second;
         } else {
            return nullptr;
         }
      }
   };

   // BUG: You need to make sure that the basic blocks aren't w/i the same loop.

   const auto do_connect = [&] (const Port& src_port, const llvm::BasicBlock *src_B,
                                const llvm::Loop *L) {
      const llvm::Instruction *T = src_B->getTerminator();
      for (unsigned succ_i = 0; succ_i < T->getNumSuccessors(); ++succ_i) {
         /* get the successor and the successor port */
         const llvm::BasicBlock *dst = T->getSuccessor(succ_i);
         if (const Port *dst_port = get_block_port(dst, L)) {
            const auto src_range = src_port.exits.equal_range(src_B);
            for (auto src_it = src_range.first; src_it != src_range.second; ++src_it) {
               add_edge(src_it->second, dst_port->entry);
            }
         }
      }
   };

   for (const auto& nonloop_pair : nonloop_block_to_port) {
      const llvm::BasicBlock *src_B = nonloop_pair.first;
      const Port& src_port = nonloop_pair.second;
      do_connect(src_port, src_B, nullptr);
   }
   
   for (const auto& loop_pair : loop_to_port) {
      const llvm::Loop *src_L = loop_pair.first;
      const Port& src_port = loop_pair.second;
      llvm::SmallVector<llvm::BasicBlock *> src_Bs;
      src_L->getExitingBlocks(src_Bs);
      for (const llvm::BasicBlock *src_B : src_Bs) {
         do_connect(src_port, src_B, src_L);
      }
   }


   /* define functional ports for this component */
   port.entry = get_block_port(LF->entry, nullptr)->entry;
   assert(port.entry);

   port.exits.clear();
   for (const llvm::BasicBlock *B : LF->exits) {
      port.exits.merge(std::move(get_block_port(B, nullptr)->exits));
   }
}

void AEGPO2::construct_loop(const llvm::Loop *L, Port& port) {
   LoopForest LF;
   LF.entry = L->getHeader();
   llvm::SmallVector<llvm::BasicBlock *> exits;
   L->getExitingBlocks(exits);
   std::copy(exits.begin(), exits.end(), std::back_inserter(LF.exits));
   std::copy(L->block_begin(), L->block_end(), std::back_inserter(LF.blocks));
   std::copy(L->begin(), L->end(), std::back_inserter(LF.loops));
   
   /* Generate all iterations */
   struct Iteration {
      Port port;
      typename Rel::Set continuations;
   };
   std::vector<Iteration> iterations;
   for (unsigned i = 0; i < num_unrolls + 1; ++i) {
      Iteration iteration;
      NodeRefSet iteration_nodes;
      construct_loop_forest(&LF, iteration.port,
                            std::inserter(iteration_nodes, iteration_nodes.end()));

      // add loop set information
      add_lf_set(iteration_nodes, loops);
      
      // remove back-edges to header, remembering in iteration continuations
      const NodeRef iteration_header = iteration.port.entry;
      iteration.continuations = po.rev.at(iteration_header);

      for (NodeRef continuation : iteration.continuations) {
         erase_edge(continuation, iteration_header);
      }

      iterations.push_back(iteration);
   }

   /* Glue iterations together */
   for (unsigned i = 0; i < num_unrolls - 1 + 1; ++i) {
      for (NodeRef continuation : iterations[i].continuations) {
         add_edge(continuation, iterations[i + 1].port.entry);
      }
   }

   /* Set global port info */
   port.entry = iterations[0].port.entry;
   for (auto& iteration : iterations) {
      port.exits.merge(std::move(iteration.port.exits));
   }
}


// NOTE: A function may be constructed multiple times.
void AEGPO2::construct_function(llvm::Function *F, Port& port) {
   const llvm::DominatorTree dom_tree {*F};
   const llvm::LoopInfo loop_info {dom_tree};
   LoopForest LF;
   LF.entry = &F->getEntryBlock();
   // exits: need to find all blocks w/ no successors
   for (const llvm::BasicBlock& B : *F) {
      const llvm::Instruction *T = B.getTerminator();
      if (T->getNumSuccessors() == 0) {
         LF.exits.push_back(&B);
      }
   }
   std::transform(F->begin(), F->end(), std::back_inserter(LF.blocks),
                  [] (const llvm::BasicBlock& B) {
                     return &B;
                  });
   std::copy(loop_info.begin(), loop_info.end(), std::back_inserter(LF.loops));
   NodeRefSet func_nodes;
   construct_loop_forest(&LF, port, std::inserter(func_nodes, func_nodes.end()));
   add_lf_set(func_nodes, funcs);
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO2::Node& node) {
   std::visit(util::overloaded {
         [&] (Entry) { os << "<ENTRY>"; },
         [&] (Exit)  { os << "<EXIT>";  },
         [&] (const llvm::Instruction *I) { os << *I; },
      }, node());
   return os;
}

void AEGPO2::dump_graph(const std::string& path) const {
   po.group().dump_graph(path, [&] (auto& os, const auto& group) {
      for (const NodeRef& ref : group) {
         os << ref << " " << lookup(ref) << "\n";
      }
   });
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


void AEGPO2::add_lf_set(const NodeRefSet& set, LFMap& map) {
   auto ptr = std::make_shared<NodeRefSet>(set);
   for (NodeRef ref : set) {
      map.emplace(ref, ptr);
   }
}


void AEGPO2::dump_lf_map(llvm::raw_ostream& os, const LFMap& map) const {
   std::unordered_set<std::shared_ptr<NodeRefSet>> sets;
   std::transform(map.begin(), map.end(), std::inserter(sets, sets.end()),
                  [] (const auto& pair) { return pair.second; });
   for (const auto& set : sets) {
      os << "{";
      for (NodeRef ref : *set) {
         os << ref << " ";
      }
      os << "}\n";
   }
}
