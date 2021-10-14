#include <deque>

#include "cfg/expanded.h"
#include "cfg/unrolled.h"
#include "cfg/calls.h"
#include "util.h"

template <typename Expand>
void CFG_Expanded::construct(const CFG& in, Expand& expand) {
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
            const NodeRef dst_tmp = add_node(in.lookup(task.in_dst));
            assert(dst == dst_tmp);
            newnode = true;
        } else {
            newnode = false;
        }
        
        const Exec exec = {
            .arch = task.fork.can_arch(),
            .trans = task.fork.can_trans(num_specs),
        };
        const auto res = execs.emplace(dst, exec);
        assert(res.first->second == exec);
        
        po.insert(task.src, dst);
        
        if (newnode) {
            for (NodeRef succ : in.po.fwd.at(task.in_dst)) {
                std::vector<Fork> new_forks;
                expand.transition(task.fork, task.in_dst, succ, std::back_inserter(new_forks));
                for (const Fork& new_fork : new_forks) {
                    queue.push_front(task_type {
                        .src = dst,
                        .in_dst = succ,
                        .fork = new_fork,
                    });
                }
            }
            
            expansions[task.in_dst].insert(dst);
        }
    }
    
    std::cerr << __FUNCTION__ << ": nodes: " << size() << "\n";
    
    /* set exit */
    for (NodeRef ref = 0; ref < size(); ++ref) {
        if (std::holds_alternative<Exit>(lookup(ref).v)) {
            exits.insert(ref);
        }
    }
    
    /* remaining tasks */
    resolve_refs(in);
}


void CFG_Expanded::resolve_single_ref(const llvm::Instruction *I, const llvm::Value *V, const CFG &in, std::unordered_map<NodeRef, RefMap> &maps, CFG::Node &node, NodeRef ref) {
    using Key = Translations::Key;
    using Map = RefMap;
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
    
    llvm::errs() << "resolving " << *I << " " << *V << "\n";
    
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
                        assert(it != map.end());
                    }
                }
                if (it != map.end()) {
                    const NodeRefSet& refs = it->second;
                    node.refs[V].insert(refs.begin(), refs.end());
                }
            }
            break;
        }
            
        case OTHER:
            break;
            
        case UNKNOWN:
            llvm::errs() << "value of unknown kind: " << *V << "\n";
            std::abort();
    }
}

template void CFG_Expanded::construct(const CFG& in, Expand_SpectreV1& expand);
template void CFG_Expanded::construct(const CFG& in, Expand_SpectreV4& expand);

void CFG_Expanded::resolve_refs(const CFG& in) {
    /* Approach
     * We want to bind all llvm::Argument's and llvm::Instruction's. We should leave other kinds of llvm::Value's alone.
     */
    
    std::vector<NodeRef> order;
    reverse_postorder(std::back_inserter(order));
    using Translations = CFG_Unrolled::Translations;
    using Key = Translations::Key;
    using Map = std::unordered_map<Key, NodeRefSet, Key::Hash>;
    std::unordered_map<NodeRef, Map> maps;
    
    for (const NodeRef ref : order) {
        // merge incoming
        Map map;
        for (const NodeRef pred : po.rev.at(ref)) {
            const Map& a = maps.at(pred);
            for (const auto& p : a) {
                map[p.first].insert(p.second.begin(), p.second.end());
            }
        }
        
        const Node& node = lookup(ref);
        
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
        
        maps[ref] = std::move(map);
    }
    
    /* Now resolve refs */
    for (const NodeRef ref : order) {
        Node& node = lookup(ref);
        node.refs.clear();
        std::visit(util::overloaded {
            [&] (const llvm::Instruction *I) {
                for (const llvm::Value *V : I->operand_values()) {
                    resolve_single_ref(I, V, in, maps, node, ref);
                }
            },
            [&] (const Node::Call& call) {
                resolve_single_ref(call.C, call.arg, in, maps, node, ref);
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
