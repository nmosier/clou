#pragma once

#include <unordered_map>
#include <unordered_set>
#include <set>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Function.h>
#include <llvm/Analysis/LoopInfo.h>

#include "util/llvm.h"

namespace dataflow {


template <class Value>
struct Dataflow {
    using InstSet = std::unordered_set<const llvm::Instruction *>;
    using Map = std::unordered_map<const llvm::Instruction *, Value>;
    using Transfer = std::function<Value (const llvm::Instruction *, const Value&)>;
    using Meet = std::function<Value (const Value&, const Value&)>;
    
    struct Component;
    
    struct Context {
        Value top;
        Transfer transfer;
        Meet meet;
    };
    
    struct Component {
        Context context;
        
        Component(const Context& context): context(context) {}
        
        virtual ~Component() = default;
        
        virtual const llvm::Instruction *entry() const = 0;
        virtual InstSet exits() const = 0;
        virtual InstSet insts() const = 0;
        virtual void transfer(Value entry_value, Map& ins, Map& outs, Map& exit_values) const = 0;
        virtual void print(llvm::raw_ostream& os, unsigned indent) const = 0;
    };
    
    struct Graph: Component {
        /** Map of instruction entries to components. */
        std::unordered_map<const llvm::Instruction *, std::unique_ptr<Component>> map;
        
        /** Edges to ignore. */
        std::set<std::pair<const llvm::Instruction *, const llvm::Instruction *>> ignore;
        
        Graph(const Context& context): Component(context) {}
        
        virtual const llvm::Instruction *entry() const override {
            return entry_;
        }
        
        virtual InstSet exits() const override {
            return exits_;
        }
        
        void entry(const llvm::Instruction *I) {
            entry_ = I;
        }
        
        void exits(const InstSet& exits) {
            exits_ = exits;
        }
        
        virtual InstSet insts() const override {
            InstSet out;
            for (const auto& p : map) {
                const auto& insts = p.second->insts();
                std::copy(insts.begin(), insts.end(), std::inserter(out, out.end()));
            }
            return out;
        }
        
        virtual void transfer(Value entry_value, Map& ins, Map& outs, Map& exit_values) const override {

            // Initialization: set interiors to top, set entry value
            for (const llvm::Instruction *I : insts()) {
                ins.insert_or_assign(I, this->context.top);
                outs.insert_or_assign(I, this->context.top);
            }
            
            Map local_exit_values;
            for (const auto& p : map) {
                const Component& component = *p.second;
                for (const llvm::Instruction *I : component.exits()) {
                    local_exit_values.insert_or_assign(I, this->context.top);
                }
            }
            
            ins.insert_or_assign(entry(), entry_value);
            
            // Repeat until no changes
            Map old_ins, old_outs, old_local_exit_values;
            const auto local_exit_values_size = local_exit_values.size();
            while (ins != old_ins || outs != old_outs || local_exit_values != old_local_exit_values) {
                old_ins = ins;
                old_outs = outs;
                old_local_exit_values = local_exit_values;
                assert(local_exit_values.size() == local_exit_values_size);
                meet_once(ins, local_exit_values);
                assert(local_exit_values.size() == local_exit_values_size);
                local_exit_values.clear();
                transfer_once(ins, outs, local_exit_values);
                assert(local_exit_values.size() == local_exit_values_size);
            }
            
            // clear out exit values
            const InstSet exits = this->exits();
            std::copy_if(local_exit_values.begin(), local_exit_values.end(), std::inserter(exit_values, exit_values.end()), [&exits] (const auto& p) -> bool {
                return exits.contains(p.first);
            });
        }
        
        virtual void print(llvm::raw_ostream& os, unsigned indent) const override {
            std::string s (indent, ' ');
            os << s << "Graph {\n";
            for (const auto& p : map) {
                p.second->print(os, indent + 2);
                os << s << "  " << "entry = " << *p.second->entry() << "\n";
                os << s << "  " << "exits = {\n";
                for (const llvm::Instruction *I : p.second->exits()) {
                    os << s << "    " << *I << "\n";
                }
                os << s << "  }\n";
            }
            os << s << "}\n";
        }
        
        static Graph from_function_instruction(const llvm::Function& F, const Context& context) {
            Graph G {context};
            G.entry(&F.getEntryBlock().front());
            for (const llvm::BasicBlock& B : F) {
                for (const llvm::Instruction& I : B) {
                    auto instruction = std::make_unique<Instruction>(context, &I);
                    G.map.insert_or_assign(&I, std::move(instruction));
                }
            }
            return G;
        }
        
        static Graph from_function_block(const llvm::Function& F, const Context& context) {
            Graph G {context};
            G.entry(&F.getEntryBlock().front());
            for (const llvm::BasicBlock& B : F) {
                Block block {context, &B};
                G.map.insert_or_assign(&B.front(), std::make_unique<Block>(block));
            }
            return G;
        }
                
    private:
        const llvm::Instruction *entry_ = nullptr;
        InstSet exits_;
        
        bool check_edge(const llvm::Instruction *src, const llvm::Instruction *dst) const {
            return component_entries().contains(dst) && component_exits().contains(src) && !ignore.contains({src, dst});
        }
        
        void meet_once(Map& ins, const Map& outs) const {
            // perform meet on all
            for (const auto& pair : map) {
                const llvm::Instruction *I = pair.first;
                const llvm::BasicBlock *B = I->getParent();
                
                auto& in = ins.at(I);
                
                if (I == &B->front()) {
                    // TODO: might not be necessary
                    if (I != entry()) {
                        in = this->context.top;
                    }
                    for (const llvm::BasicBlock *B_pred : llvm::predecessors(B)) {
                        const llvm::Instruction *I_pred = B_pred->getTerminator();
                        if (check_edge(I_pred, I)) {
                            in = this->context.meet(in, outs.at(I_pred));
                        }
                    }
                } else {
                    
                    const llvm::Instruction *I_pred = I->getPrevNode();
                    assert(check_edge(I_pred, I));
                    in = outs.at(I_pred);
                    
                }
                
            }
        }
        
        void transfer_once(Map& ins, Map& outs, Map& exit_values) const {
            for (const llvm::Instruction *I : order()) {
                const Component& component = *map.at(I);
                Map component_exit_values;
                component.transfer(ins.at(component.entry()), ins, outs, component_exit_values);
                std::copy(component_exit_values.begin(), component_exit_values.end(), std::inserter(exit_values, exit_values.end()));
            }
        }
        
        InstSet component_exits() const {
            InstSet out;
            for (const auto& p : map) {
                const auto& exits = p.second->exits();
                std::copy(exits.begin(), exits.end(), std::inserter(out, out.end()));
            }
            return out;
        }
        
        InstSet component_entries() const {
            InstSet out;
            for (const auto& p : map) {
                out.insert(p.second->entry());
            }
            return out;
        }
        
        std::vector<const llvm::Instruction *> order() const {
            std::vector<const llvm::Instruction *> out;
            if (!map.empty()) {
                const llvm::Function& F = *map.begin()->first->getFunction();
                for (const llvm::BasicBlock& B : F) {
                    for (const llvm::Instruction& I : B) {
                        if (map.contains(&I)) {
                            out.push_back(&I);
                        }
                    }
                }
            }
            return out;
        }
    };
    
    struct Instruction: Component {
        const llvm::Instruction *I;
        
        Instruction(const Context& context, const llvm::Instruction *I): Component(context), I(I) {}
        
        virtual const llvm::Instruction *entry() const override { return I; }
        virtual InstSet exits() const override { return {I}; }
        virtual InstSet insts() const override { return {I}; }
        
        virtual void transfer(Value entry_value, Map& ins, Map& outs, Map& exit_values) const override {
            ins.insert_or_assign(I, entry_value);
            const Value out = this->context.transfer(entry(), entry_value);
            outs.insert_or_assign(I, out);
            exit_values.insert_or_assign(I, out);
            
#if 0
            llvm::errs() << "Transfer for " << *I << ":\n";
            llvm::errs() << "IN:\n" << entry_value;
            llvm::errs() << "OUT:\n" << exit_values.at(I) << "\n";
            llvm::errs() << "\n";
#endif
        }
        
        virtual void print(llvm::raw_ostream& os, unsigned indent) const override {
            std::string s (indent, ' ');
            os << s << "Instruction { " << *I << " }\n";
        }
        
    };
    
    struct Block: Component {
        const llvm::BasicBlock *B;
        
        Block(const Context& context, const llvm::BasicBlock *B): Component(context), B(B) {}
        
        virtual const llvm::Instruction *entry() const override { return &B->front(); }
        virtual InstSet exits() const override { return {&B->back()}; }
        InstSet insts() const override {
            InstSet out;
            for (const llvm::Instruction& I : *B) {
                out.insert(&I);
            }
            return out;
        }
        
        virtual void transfer(Value entry_value, Map& ins, Map& outs, Map& exit_values) const override {
            Value in = entry_value;
            for (const llvm::Instruction& I : *B) {
                Instruction instruction {this->context, &I};
                Map out;
                instruction.transfer(in, ins, outs, out);
                assert(out.size() == 1);
                in = out.begin()->second;
            }
            
            exit_values.insert_or_assign(B->getTerminator(), in);
        }
        
        virtual void print(llvm::raw_ostream& os, unsigned indent) const override {
            std::string s (indent, ' ');
            os << s << "Block {\n";
            for (const llvm::Instruction& I : *B) {
                os << s << "  " << I << "\n";
            }
            os << s << "}\n";
        }
    };
    
    struct Loop: Component {
        const llvm::Loop *L;
        
        Loop(const Context& context, const llvm::Loop *L): Component(context), L(L) {}
        
        virtual const llvm::Instruction *entry() const override {
            return &L->getHeader()->front();
        }
        
        virtual void print(llvm::raw_ostream& os, unsigned indent) const override {
            std::string s (indent, ' ');
            os << s << "Loop {\n";
            const Graph graph = unconstrained_graph();
            graph.print(os, indent + 2);
            os << s << "\n";
        }
        
        virtual InstSet exits() const override {
            InstSet exit_insts;
            llvm::SmallVector<llvm::BasicBlock *, 16> exit_blocks;
            L->getExitingBlocks(exit_blocks);
            for (const llvm::BasicBlock *B : exit_blocks) {
                exit_insts.insert(B->getTerminator());
            }
            return exit_insts;
        }
        
        virtual InstSet insts() const override {
            InstSet out;
            for (const llvm::BasicBlock *B : L->blocks()) {
                for (const llvm::Instruction& I : *B) {
                    out.insert(&I);
                }
            }
            return out;
        }
        
        virtual void transfer(Value entry_value, Map& ins, Map& outs, Map& exit_values) const override {
            
            const Value old_entry_value = entry_value;
            
            Graph graph = unconstrained_graph();
            graph.entry(entry());
#if 1
            transfer_internal(graph, entry_value, ins, outs);
            assert(entry_value == old_entry_value);
            transfer_external(graph, entry_value, exit_values);
#else
            graph.transfer(entry_value, ins, outs, exit_values);
#endif
        }
        
    private:
        Graph unconstrained_graph() const {
            Graph graph {this->context};
            
            std::unordered_set<const llvm::BasicBlock *> loop_blocks;
            for (const llvm::Loop *subloop : *L) {
                Loop loop {this->context, subloop};
                graph.map.insert_or_assign(loop.entry(), std::make_unique<Loop>(loop));
                std::copy(subloop->block_begin(), subloop->block_end(), std::inserter(loop_blocks, loop_blocks.end()));
            }
            
            for (const llvm::BasicBlock *B : L->blocks()) {
                if (!loop_blocks.contains(B)) {
                    Block block {this->context, B};
                    graph.map.insert_or_assign(block.entry(), std::make_unique<Block>(block));
                }
            }

            return graph;
        }
        
        bool is_backblock(const llvm::BasicBlock *B) const {
            const llvm::BasicBlock *H = L->getHeader();
            for (const llvm::BasicBlock *B_succ : llvm::successors(B)) {
                if (B_succ == H) {
                    return true;
                }
            }
            return false;
        }
        
        std::unordered_set<const llvm::BasicBlock *> backblocks() const {
            std::unordered_set<const llvm::BasicBlock *> backblocks;
            std::copy_if(L->block_begin(), L->block_end(), std::inserter(backblocks, backblocks.end()), [&] (const llvm::BasicBlock *B) -> bool {
                return is_backblock(B);
            });
            assert(!backblocks.empty());
            return backblocks;
        }
        
        void transfer_internal(Graph& graph, const Value& entry_value, Map& ins, Map& outs) const {
            graph.exits(InstSet()); // clear exits
            Map exit_values;
            graph.transfer(entry_value, ins, outs, exit_values);
        }
        
        void transfer_external(Graph& graph, const Value& entry_value, Map& exit_values) const {
            
            
            // ignore back edges
            const auto backblocks = this->backblocks();
            for (const llvm::BasicBlock *B : backblocks) {
                graph.ignore.emplace(B->getTerminator(), entry());
            }
            
            const unsigned min_loop_iterations = llvm::get_min_loop_iterations(L);
            
            /* For min_loop_iterations:
             * 1a Transfer the constrained graph.
             * 1b Meet the exit values of backblocks to get new entry.
             * 2 Do final pass.
             */
            InstSet backinsts;
            std::transform(backblocks.begin(), backblocks.end(), std::inserter(backinsts, backinsts.end()), [] (const llvm::BasicBlock *B) -> const llvm::Instruction * {
                return B->getTerminator();
            });
            graph.exits(backinsts);
            Value in = entry_value;
            for (unsigned i = 0; i < min_loop_iterations; ++i) {
                // transfer
                Map ins, outs, exit_values;
                graph.transfer(in, ins, outs, exit_values);
                
                // meet exit values
                in = this->context.top;
                for (const llvm::Instruction *I : backinsts) {
                    in = this->context.meet(in, exit_values.at(I));
                }
            }
            
            // final pass
            graph.exits(exits());
            graph.ignore.clear();
            {
                Map ins, outs;
                graph.transfer(in, ins, outs, exit_values);
            }
        }
        
    };
    
    
    struct Function: Component {
        enum class Mode {
            INST,
            BLOCK,
            LOOP,
        };
        
        const llvm::Function *F;
        const llvm::LoopInfo *LI;
        Mode mode;
        
        Function(const Context& context, const llvm::Function *F, const llvm::LoopInfo *LI, Mode mode): Component(context), F(F), LI(LI), mode(mode) {}
        
        virtual void print(llvm::raw_ostream& os, unsigned indent) const override {
            std::string s (indent, ' ');
            os << s << "Function {\n";
            get_graph().print(os, indent + 2);
            os << s << "\n";
        }
        
        virtual const llvm::Instruction *entry() const override {
            return &F->getEntryBlock().front();
        }
        
        virtual InstSet exits() const override {
            return {};
        }
        
        virtual InstSet insts() const override {
            InstSet out;
            for (const llvm::BasicBlock& B : *F) {
                for (const llvm::Instruction& I : B) {
                    out.insert(&I);
                }
            }
            return out;
        }
        
        virtual void transfer(Value entry_value, Map& ins, Map& outs, Map& exit_values) const override {
            static const std::unordered_map<Mode, void (Function::*)(const Value&, Map&, Map&, Map&) const> map = {
                {Mode::INST, &Function::transfer_inst},
                {Mode::BLOCK, &Function::transfer_block},
                {Mode::LOOP, &Function::transfer_loop},
            };
            (this->*map.at(mode))(entry_value, ins, outs, exit_values);
        }
        
    private:
        void transfer_inst(const Value& entry_value, Map& ins, Map& outs, Map& exit_values) const {
            Graph graph {this->context};
            graph.entry(entry());
            for (const llvm::BasicBlock& B : *F) {
                for (const llvm::Instruction& I : B) {
                    Instruction instruction {this->context, &I};
                    graph.map.insert_or_assign(instruction.entry(), std::make_unique<Instruction>(instruction));
                }
            }
            graph.transfer(entry_value, ins, outs, exit_values);
        }
        
        void transfer_block(const Value& entry_value, Map& ins, Map& outs, Map& exit_values) const {
            Graph graph {this->context};
            graph.entry(entry());
            for (const llvm::BasicBlock& B : *F) {
                Block block {this->context, &B};
                graph.map.insert_or_assign(block.entry(), std::make_unique<Block>(block));
            }
            graph.transfer(entry_value, ins, outs, exit_values);
        }
        
        void transfer_loop(const Value& entry_value, Map& ins, Map& outs, Map& exit_values) const {
            Graph graph {this->context};
            graph.entry(entry());
            
            std::unordered_set<const llvm::BasicBlock *> loop_blocks;
            for (const llvm::Loop *L : *LI) {
                std::copy(L->block_begin(), L->block_end(), std::inserter(loop_blocks, loop_blocks.end()));
                Loop loop {this->context, L};
                graph.map.insert_or_assign(loop.entry(), std::make_unique<Loop>(loop));
            }
            
            for (const llvm::BasicBlock& B : *F) {
                if (!loop_blocks.contains(&B)) {
                    Block block {this->context, &B};
                    graph.map.insert_or_assign(block.entry(), std::make_unique<Block>(block));
                }
            }
            
            graph.transfer(entry_value, ins, outs, exit_values);
        }
        
        Graph get_graph() const {
            Graph graph {this->context};
            graph.entry(entry());
            
            std::unordered_set<const llvm::BasicBlock *> loop_blocks;
            for (const llvm::Loop *L : *LI) {
                std::copy(L->block_begin(), L->block_end(), std::inserter(loop_blocks, loop_blocks.end()));
                Loop loop {this->context, L};
                graph.map.insert_or_assign(loop.entry(), std::make_unique<Loop>(loop));
            }
            
            for (const llvm::BasicBlock& B : *F) {
                if (!loop_blocks.contains(&B)) {
                    Block block {this->context, &B};
                    graph.map.insert_or_assign(block.entry(), std::make_unique<Block>(block));
                }
            }
            
            return graph;
        }
        
    };
    
};

}
