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
}

void AEGPO2::construct_instruction(const llvm::Instruction *I, Port& port) {
   // TODO: Should handle function calls.
   
   port.entry = add_node(Node::make(I));
   port.exits = {{I->getParent(), port.entry}};
}

void AEGPO2::construct_block(const llvm::BasicBlock *B, Port& port) {
   std::vector<Port> ports;
   ports.resize(B->size());

   auto port_it = ports.begin();
   for (const llvm::Instruction& I : *B) {
      construct_instruction(&I, *port_it);
      if (port_it != ports.begin()) {
         auto port_prev = std::next(port_it, -1);
         connect(port_prev->exits.begin(), port_prev->exits.end(), port_it->entry);
      }
      ++port_it;
   }
   
   port.entry = ports.front().entry;
   port.exits = std::move(ports.back().exits);
}

void AEGPO2::construct_loop_forest(const LoopForest *LF, Port& port) {
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
         construct_block(B, B_port);
         nonloop_block_to_port.emplace(B, B_port);
      }
   }

   /* connect ports */
   const auto get_block_port = [&] (const llvm::BasicBlock *succ) -> Port * {
      const auto succ_loop_it = block_to_loop.find(succ);
      if (succ_loop_it != block_to_loop.end()) {
         return &loop_to_port.at(succ_loop_it->second);
      } else {
         const auto nonloop_it = nonloop_block_to_port.find(succ);
         if (nonloop_it != nonloop_block_to_port.end()) {
            return &nonloop_it->second;
         } else {
            return nullptr;
         }
      }
   };

   const auto do_connect = [&] (const Port& src_port, const llvm::BasicBlock *src_B) {
      const llvm::Instruction *T = src_B->getTerminator();
      for (unsigned succ_i = 0; succ_i < T->getNumSuccessors(); ++succ_i) {
         /* get the successor and the successor port */
         const llvm::BasicBlock *dst = T->getSuccessor(succ_i);
         if (const Port *dst_port = get_block_port(dst)) {
            add_edge(src_port.exits.at(src_B), dst_port->entry);
         }
      }
   };

   for (const auto& nonloop_pair : nonloop_block_to_port) {
      const llvm::BasicBlock *src_B = nonloop_pair.first;
      const Port& src_port = nonloop_pair.second;
      do_connect(src_port, src_B);
   }
   
   for (const auto& loop_pair : loop_to_port) {
      const llvm::Loop *src_L = loop_pair.first;
      const Port& src_port = loop_pair.second;
      llvm::SmallVector<llvm::BasicBlock *> src_Bs;
      src_L->getExitingBlocks(src_Bs);
      for (const llvm::BasicBlock *src_B : src_Bs) {
         do_connect(src_port, src_B);
      }
   }


   /* define functional ports for this component */
   port.entry = get_block_port(LF->entry)->entry;
   assert(port.entry);

   port.exits.clear();
   for (const llvm::BasicBlock *B : LF->exits) {
      port.exits.merge(std::move(get_block_port(B)->exits));
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
   for (unsigned i = 0; i < num_unrolls; ++i) {
      Iteration iteration;
      construct_loop_forest(&LF, iteration.port);

      // remove back-edges to header, remembering in iteration continuations
      const NodeRef iteration_header = iteration.port.entry;
      iteration.continuations = po.rev.at(iteration_header);
      for (NodeRef continuation : iteration.continuations) {
         erase_edge(continuation, iteration_header);
      }

      iterations.push_back(iteration);
   }

   /* Glue iterations together */
   for (unsigned i = 0; i < num_unrolls - 1; ++i) {
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
   construct_loop_forest(&LF, port);
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const AEGPO2::Node& node) {
   std::visit(util::overloaded {
         [&] (AEGPO2::Node::Entry) { os << "<ENTRY>"; },
         [&] (AEGPO2::Node::Exit)  { os << "<EXIT>";  },
         [&] (const llvm::Instruction *I) { os << *I; },
      }, node());
   return os;
}

void AEGPO2::dump_graph(const std::string& path) const {
   po.group().dump_graph(path, [&] (auto& os, const auto& group) {
      for (const NodeRef& ref : group) {
         os << lookup(ref) << "\n";
      }
   });
}

