#include <llvm/IR/Dominators.h>

#include "aeg-po2.h"

/* Construction Algorithm
 * We will construct functions recursively. Constructing a function should return the
 * entry and exit nodes.
 * While constructing each function, we will then proceed with the regular flow. 
 *
 */

void AEGPO2::construct() {
   std::abort(); // TODO
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

void AEGPO2::construct_loop(const llvm::Loop *L, Port& port) {
   /* construct header */
   Port header_port;
   construct_block(L->getHeader(), header_port);
   port.entry = header_port.entry;

   /*
    * We can't just have a vector of NodeRefs be the exits.
    * It must be pairs of NodeRefs with their corresponding BASIC BLOCKS.
    *
    */

   /* construct loops */
   std::unordered_map<const llvm::BasicBlock *, const llvm::Loop *> block_to_loop;
   std::unordered_map<const llvm::Loop *, Port> loop_to_port;
   for (const llvm::Loop *subL : L->getSubLoops()) {
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
   for (const llvm::BasicBlock *B : L->blocks()) {
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
         return &nonloop_block_to_port.at(succ);
      }
   };

   const auto do_connect = [&] (const Port& src_port, const llvm::BasicBlock *src_B) {
      const llvm::Instruction *T = src_B->getTerminator();
      for (unsigned succ_i = 0; succ_i < T->getNumSuccessors(); ++succ_i) {
         /* get the successor and the successor port */
         const llvm::BasicBlock *dst = T->getSuccessor(succ_i);
         const Port *dst_port = get_block_port(dst);
         add_edge(src_port.exits.at(dst), dst_port->entry);
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
      src_L->getExitBlocks(src_Bs);
      for (const llvm::BasicBlock *src_B : src_Bs) {
         do_connect(src_port, src_B);
      }
   }


   /* define functional ports for this component */
   port.entry = get_block_port(L->getHeader())->entry;

   port.exits.clear();
   llvm::SmallVector<llvm::BasicBlock *> Bs;
   L->getExitBlocks(Bs);
   for (const llvm::BasicBlock *B : Bs) {
      port.exits.merge(std::move(get_block_port(B)->exits));
   }
}



// NOTE: A function may be constructed multiple times.
void AEGPO2::construct_function(llvm::Function *F, Port& port) {
   const llvm::DominatorTree dom_tree {*F};
   const llvm::LoopInfo loop_info {dom_tree};
   
}
