#include <deque>

#include "cfg/expanded.h"
#include "cfg/unrolled.h"
#include "cfg/calls.h"
#include "util.h"
#include "spec-prim.h"

void AEGPO_Expanded::construct(const AEGPO& in, const SpeculationInfo& spec) {
    // create entry
    NodeRef in_src = in.entry;
    NodeRef src = add_node(in.lookup(in_src));
    entry = src;
    NodeMap map {{in_src, src}};
    
    std::deque<Task> queue;
    for (NodeRef in_dst : in.po.fwd.at(in_src)) {
        queue.push_back(Task {
            .in_src = in_src,
            .in_dst = in_dst,
            .src = src,
            .spec_depth = num_specs,
            .forks = {{.always_speculative = false}},
        }); // since ENTRY will never be speculatively executed
    }
    while (!queue.empty()) {
        const Task& task = queue.back();
        construct_rec(in, spec, task, map, std::front_inserter(queue));
        queue.pop_back();
    }
    
    /* set exit */
    for (NodeRef ref = 0; ref < size(); ++ref) {
        if (std::holds_alternative<Exit>(lookup(ref).v)) {
            exits.insert(ref);
        }
    }
    assert(!exits.empty());
    
    // DEBUG: print added nodes
    for (const auto& pair : expansions) {
        if (pair.second.size() > 1) {
            logv(4) << "expanded node " << pair.second.size() << " times: " << in.lookup(pair.first) << "\n";
        }
    }
    
    // remaining tasks
    resolve_refs(in);
}

/* NOTE: spec_depth is the depth of in_src.
 *
 */
template <typename OutputIt>
void AEGPO_Expanded::construct_rec(const AEGPO& in, const SpeculationInfo& spec, const Task& task, NodeMap& map, OutputIt out) {

    /* check whether to merge or duplicate node */
    const unsigned spec_depth = task.spec_depth + 1;
    
    for (const Fork& fork : task.forks) {
        bool newnode;
        NodeRef dst;
        
        if (spec_depth <= num_specs) {
            /* create private node */
            std::cerr << "private " << task.in_dst << "\n";
            dst = add_node(in.lookup(task.in_dst));
            newnode = true;
        } else if (fork.always_speculative) {
            // this fork is dead, so just continue
            continue;
        } else {
            /* merge with or create public node */
            const auto it = map.find(task.in_dst);
            if (it == map.end()) {
                // create
                dst = add_node(in.lookup(task.in_dst));
                map.emplace(task.in_dst, dst);
                newnode = true;
            } else {
                dst = it->second;
                newnode = false;
            }
        }
        
        po.insert(task.src, dst);
        
        if (newnode) {
            const NodeRef next_in_src = task.in_dst;
            for (NodeRef next_in_dst : in.po.fwd.at(next_in_src)) {
                unsigned new_spec_depth;
                std::vector<Fork> forks;
                const auto it = spec.primitive_tfos.find(std::make_tuple(next_in_src, next_in_dst));
                if (fork.always_speculative || it == spec.primitive_tfos.end()) {
                    new_spec_depth = spec_depth;
                    forks.push_back({.always_speculative = fork.always_speculative});
                } else {
                    new_spec_depth = 0;
                    forks = it->second.forks;
                }
                
                *out++ = Task {
                    .in_src = next_in_src,
                    .in_dst = next_in_dst,
                    .src = dst,
                    .spec_depth = new_spec_depth,
                    .forks = forks,
                };
            }
            expansions[task.in_dst].insert(dst);
        }
        
    }
}

void AEGPO_Expanded::resolve_single_ref(const llvm::Instruction *I, const llvm::Value *V, const AEGPO &in, std::unordered_map<NodeRef, RefMap> &maps, AEGPO::Node &node, NodeRef ref) {
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

void AEGPO_Expanded::resolve_refs(const AEGPO& in) {
    /* Approach
     * We want to bind all llvm::Argument's and llvm::Instruction's. We should leave other kinds of llvm::Value's alone.
     */
    
    std::vector<NodeRef> order;
    reverse_postorder(std::back_inserter(order));
    using Translations = AEGPO_Unrolled::Translations;
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
