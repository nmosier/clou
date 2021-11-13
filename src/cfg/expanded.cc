#include <deque>
#include <gperftools/profiler.h>

#include <llvm/IR/InlineAsm.h>

#include "cfg/expanded.h"
#include "cfg/unrolled.h"
#include "cfg/calls.h"
#include "util/output.h"
#include "util/algorithm.h"
#include "timer.h"

template <typename Expand>
void CFG_Expanded::construct_full(const CFG& in, Expand& expand) {
    using Fork = typename Expand::Fork;
    
    /* create entry */
    entry = add_node(in.lookup(in.entry));
    expansions[in.entry].insert(entry);
    execs.emplace(entry, Exec {
        .arch = Option::MUST,
        .trans = Option::NO,
    });
    
    using task_type = Task<Fork>;
    std::deque<task_type> queue;
    for (NodeRef in_dst : in.po.fwd.at(in.entry)) {
        queue.push_back(task_type {
            .in_dst = in_dst,
            .src    = entry,
            .fork  = expand.init(),
        });
    }
    
    while (!queue.empty()) {
        const task_type task = std::move(queue.back());
        queue.pop_back();
        
        bool newnode;
        
        const NodeRef end_ref = size();
        const NodeRef dst = expand.merge(task.fork, task.in_dst, end_ref);
        if (dst == end_ref) {
            // create new node
            [[maybe_unused]] const NodeRef dst_tmp = add_node(in.lookup(task.in_dst));
            assert(dst == dst_tmp);
            newnode = true;
        } else {
            newnode = false;
        }
        
        const Exec exec = {
            .arch = task.fork.can_arch(),
            .trans = task.fork.can_trans(num_specs),
        };
        [[maybe_unused]] const auto res = execs.emplace(dst, exec);
        assert(res.first->second == exec);
        
        po.insert(task.src, dst);
        
        if (newnode) {
            for (NodeRef succ : in.po.fwd.at(task.in_dst)) {
                std::vector<Fork> new_forks;
                expand.transition(task.fork, task.in_dst, succ, std::back_inserter(new_forks));
                for (const Fork& new_fork : new_forks) {
                    queue.push_front(task_type {
                        .in_dst = succ,
                        .src = dst,
                        .fork = new_fork,
                    });
                }
            }
            
            expansions[task.in_dst].insert(dst);
        }
    }
}

template <typename Expand>
void CFG_Expanded::construct_partial(const CFG& in, Expand& expand) {
    nodes = in.nodes;
    po    = in.po;
    entry = in.entry;
    exits = in.exits;

    for (NodeRef ref = 0; ref < size(); ++ref) {
        expansions.emplace(ref, NodeRefSet {ref});
        execs.emplace(ref, Exec {
            .arch = Option::MAY,
            .trans = Option::MAY,
        });
    }
}

template <typename Expand>
void CFG_Expanded::construct(const CFG& in, Expand& expand) {
    if (partial_executions) {
        construct_partial(in, expand);
    } else {
        construct_full(in, expand);
    }

    
    std::cerr << __FUNCTION__ << ": nodes: " << size() << "\n";
    
    /* set exit */
    for (NodeRef ref = 0; ref < size(); ++ref) {
        if (std::holds_alternative<Exit>(lookup(ref).v)) {
            exits.insert(ref);
        }
    }
    
    /* remaining tasks */
    {
        std::cerr << "resolving refs...\n";
        Timer timer;
        resolve_refs(in);
    }
}


void CFG_Expanded::resolve_single_ref(const llvm::Instruction *I, const llvm::Value *V, const CFG &in, std::vector<RefMap1> &maps, CFG::Node &node, NodeRef ref) {
    using Key = Translations::Key;
    using Map = RefMap1;
    enum ValueKind {
        INST, ARG, OTHER, UNKNOWN
    } kind;
    if (llvm::isa<llvm::Instruction>(V)) {
        kind = INST;
    } else if (llvm::isa<llvm::Argument>(V)) {
        kind = ARG;
    } else if (llvm::isa<llvm::Constant>(V) || llvm::isa<llvm::BasicBlock>(V)) {
        kind = OTHER;
    } else {
        kind = UNKNOWN;
    }
    
    switch (kind) {
        case INST:
        case ARG: {
            std::vector<Key> sources;
            in.translations.lookup(Key {node.id->func, V}, std::back_inserter(sources));
            
            const Map& map = maps.at(ref);
            for (const Key& source : sources) {
                const auto it = map.find(source);
                if (kind == INST) {
                    if (!llvm::isa<llvm::PHINode>(I)) {
                        if (it == map.end()) {
                            llvm::errs() << "source: " << source << "\n";
                            std::cerr << "map: ";
                            for (const auto& p : map) {
                                llvm::errs() << p.first << "\n";
                            }
                        }
                        assert(it != map.end());
                    }
                }
                if (it != map.end()) {
                    const auto& refs = (*it).second;
                    node.refs[V].insert(refs.begin(), refs.end());
                }
            }
            break;
        }
            
        case OTHER:
            break;
            
        case UNKNOWN:
            std::cerr << __FILE__ << ": " << __LINE__ << ": value of unknown kind: " << *V << "\n";
            std::cerr << "in instruction: " << *I << "\n";
            std::cerr << "in function: " << *I->getFunction() << "\n";
            std::abort();
    }
}

template void CFG_Expanded::construct(const CFG& in, Expand_SpectreV1& expand);
template void CFG_Expanded::construct(const CFG& in, Expand_SpectreV4& expand);

void CFG_Expanded::resolve_refs(const CFG& in) {
    /* Approach
     * We want to bind all llvm::Argument's and llvm::Instruction's. We should leave other kinds of llvm::Value's alone.
     */
    
    /*
     * Optimiaztion idea: remove from map once all successors have been processed
     */
    
    NodeRefVec order;
    reverse_postorder(std::back_inserter(order));
    
    using Translations = CFG_Unrolled::Translations;
    using Key = Translations::Key;
    using Map = RefMap1; // TODO: remove this unnecessary definition
    
    
/* Idea: use a string-table-like approach. Rather than the map's values being a NodeRefSet, it is an index into a table of NodeRefSets.
 */
    std::vector<Map> maps(size(), Map(size()));

    NodeRefSet done;
    for (const NodeRef ref : order) {
        done.insert(ref);

        // merge incoming
        Map& map = maps[ref];
        for (const NodeRef pred : po.rev.at(ref)) {
            Map& a = maps.at(pred);

            // check this predecessor will be done (all successors processed)
            if (util::subset(po.fwd.at(pred), done)) {
                // merge container
                map.merge(a);
            } else if (map.empty()) {
                map = a;
            } else {
                for (const auto& p : a) {
                    map[p.first].insert(p.second.begin(), p.second.end());
                }
            }
            
            // check if done
            if (util::subset(po.fwd.at(pred), done)) {
                maps.at(pred).clear();
            }
        }

        Node& node = lookup(ref);
        
        std::visit(util::overloaded {
            [&] (const llvm::Instruction *I) {
                map[Key {node.id->func, I}].insert(ref);
            },
            [&] (const Node::Call& call) {
                map[Key {node.id->func, call.C}].insert(ref);
            },
            [] (Entry) {},
            [] (Exit) {},
        }, node.v);
        
        /* resolve refs */
        node.refs.clear();
        std::visit(util::overloaded {
            [&] (const llvm::Instruction *I) {
                for (const llvm::Value *V : I->operand_values()) {
                    resolve_single_ref(I, V, in,
                                       maps,
                                       node, ref);
                }
            },
            [&] (const Node::Call& call) {
                resolve_single_ref(call.C, call.arg, in,
                                   maps,
                                   node, ref);
            },
            [&] (Entry) {},
            [&] (Exit) {},
        }, node.v);
    }
}

NodeRef Expand_SpectreV1::merge(const Fork& fork, NodeRef in_ref, NodeRef out_ref) {
    if (fork.spec_depth <= num_specs) {
        // private
        return out_ref;
    } else {
        // public
        // NOTE: This will insert node if missing and tell caller to instantiate a fresh one.
        const auto res = public_map.emplace(in_ref, out_ref);
        return res.first->second;
    }
}

NodeRef Expand_SpectreV4::merge(const Fork& fork, NodeRef in_ref, NodeRef out_ref) {
    Map *map;
    if (fork.always_speculative) {
        map = &speculative_map;
    } else {
        map = &nonspeculative_map;
    }
    const auto res = map->emplace(in_ref, out_ref);
    return res.first->second;
}
