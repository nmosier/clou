#include <deque>
#include <fstream>

#include <llvm/IR/Dominators.h>
#include <llvm/IR/Operator.h>
#include <llvm/IR/CFG.h>

#include "cfg/unrolled.h"
#include "util/scope.h"
#include "config.h"
#include "cfg/node.h"

void CFG_Unrolled::construct() {
    entry = add_node(Node::make_entry());
    NodeRef exit = add_node(Node::make_exit());
    exits = {exit};
    Port port;
    IDs ids;
    construct_function(&F, port, ids);
    add_edge(entry, port.entry);
    for (const auto& exit_pair : port.exits) {
        add_edge(exit_pair.second, exit);
    }
    prune();
}

void CFG_Unrolled::construct_instruction(const llvm::Instruction *I, Port& port, IDs& ids) {;
    if (const llvm::CallBase *C = llvm::dyn_cast<llvm::CallBase>(I)) {
        if (construct_call(C, port, ids)) {
            return;
        }
    }
    
    const Node node {I, ids.id};
    port.entry = add_node(node);
    port.exits = {{I->getParent(), port.entry}};
}

bool CFG_Unrolled::construct_call(const llvm::CallBase *C, Port& port, IDs& ids) {
    llvm::Function *F = C->getCalledFunction();
    if (F == nullptr || F->isDeclaration()) {
        return false;
    }
    
    // check if we've reached the inlining limit
    if (std::count(callstack.begin(), callstack.end(), F) > recursive_call_limit) {
        return false;
    }
    
    const std::vector<FuncID> caller_id = ids.id.func;
    ids.push();
    const std::vector<FuncID> callee_id = ids.id.func;
    
    /* add parameter translations */
    {
#if 0
        assert(F->arg_size() == C->arg_size());
#else
        assert(F->arg_size() <= C->arg_size());
#endif
        auto FA_it = F->arg_begin();
        auto CA_it = C->arg_begin();
        for (; FA_it != F->arg_end(); ++FA_it, ++CA_it) {
            const Translations::Key key {callee_id, &*FA_it};
            const Translations::Value value {caller_id, {*CA_it}};
            translations.map.emplace(key, value);
        }
    }
    
    construct_function(F, port, ids);
    ids.pop();
    
    /* add return value translations */
    {
        const Translations::Key key {caller_id, C};
        Translations::Value value {callee_id};
        if (!C->getType()->isVoidTy()) {
            for (const auto& exit : port.exits) {
                if (const llvm::ReturnInst *R = llvm::dyn_cast<llvm::ReturnInst>(&exit.first->back())) {
                    if (const llvm::Value *RV = R->getReturnValue()) {
                        if (llvm::isa<llvm::Instruction>(RV) || llvm::isa<llvm::Argument>(RV)) {
                            value.Vs.insert(RV);
                        }
                    }
                }
            }
        }
        translations.map.emplace(key, value);
        
        
    }
    
    return true;
}

void CFG_Unrolled::construct_block(const llvm::BasicBlock *B, Port& port, IDs& ids) {
    if (llvm::predecessors(B).empty() && B != &B->getParent()->getEntryBlock()) {
        return;
    }
    
    std::vector<Port> ports;
    ports.resize(B->size());
    
    auto port_it = ports.begin();
    for (const llvm::Instruction& I : *B) {
        construct_instruction(&I, *port_it, ids);
        if (port_it != ports.begin()) {
            auto port_prev = std::next(port_it, -1);
            connect(port_prev->exits.begin(), port_prev->exits.end(), port_it->entry);
        }
        ++port_it;
    }
    
    port.entry = ports.front().entry;
    port.exits = std::move(ports.back().exits);
}

void CFG_Unrolled::construct_loop_forest(const LoopForest *LF, Port& port, IDs& ids) {
    /* construct loops */
    std::unordered_map<const llvm::BasicBlock *, const llvm::Loop *> block_to_loop;
    std::unordered_map<const llvm::Loop *, Port> loop_to_port;
    for (const llvm::Loop *subL : LF->loops) {
        /* process loop */
        Port subL_port;
        construct_loop(subL, subL_port, ids);
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
            construct_block(B, B_port, ids);
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
        llvm::SmallVector<llvm::BasicBlock *, 16> src_Bs;
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

void CFG_Unrolled::construct_loop(const llvm::Loop *L, Port& port, IDs& ids) {
    LoopForest LF;
    LF.entry = L->getHeader();
    llvm::SmallVector<llvm::BasicBlock *, 16> exits;
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
        // set loop id
        ids.id.loop.push_back(ids.next_loop++);
        
        Iteration iteration;
        NodeRefSet iteration_nodes;
        construct_loop_forest(&LF, iteration.port, ids);
        
        // remove back-edges to header, remembering in iteration continuations
        const NodeRef iteration_header = iteration.port.entry;
        iteration.continuations = po.rev.at(iteration_header);
        
        for (NodeRef continuation : iteration.continuations) {
            erase_edge(continuation, iteration_header);
        }
        
        iterations.push_back(iteration);
        
        ids.id.loop.pop_back();
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
void CFG_Unrolled::construct_function(llvm::Function *F, Port& port, IDs& ids) {
    const auto push = util::push(callstack, F);
    // check if this function has already been called more than twice
#if 0
    if (std::count(callstack.begin(), callstack.end(), F) > recursive_call_limit) {
        return;
    }
#endif
    
    const llvm::DominatorTree dom_tree {*F};
    const llvm::LoopInfo loop_info {dom_tree};
    LoopForest LF;
    LF.entry = &F->getEntryBlock();
    // exits: need to find all blocks w/ no successors
    for (const llvm::BasicBlock& B : *F) {
        const llvm::Instruction *T = B.getTerminator();
        if (T->getNumSuccessors() == 0) {
            if (llvm::isa<llvm::ReturnInst>(T) || llvm::isa<llvm::UnreachableInst>(T)) {
                LF.exits.push_back(&B);
            } else {
                llvm::errs() << "unrecognized terminator instruction: " << *T << "\n";
                std::abort();
            }
        }
    }
    std::transform(F->begin(), F->end(), std::back_inserter(LF.blocks),
                   [] (const llvm::BasicBlock& B) {
        return &B;
    });
    std::copy(loop_info.begin(), loop_info.end(), std::back_inserter(LF.loops));
    NodeRefSet func_nodes;
    construct_loop_forest(&LF, port, ids);
}
