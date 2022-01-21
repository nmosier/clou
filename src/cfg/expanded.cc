#include <deque>
#include <gperftools/profiler.h>

#include <llvm/IR/InlineAsm.h>

#include "cfg/expanded.h"
#include "cfg/unrolled.h"
#include "cfg/calls.h"
#include "util/output.h"
#include "util/algorithm.h"
#include "util/timer.h"
#include "cfg/node.h"

void CFG_Expanded::construct_partial(const CFG& in) {
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

void CFG_Expanded::construct(const CFG& in) {
    assert(partial_executions);
    construct_partial(in);


    
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


void CFG_Expanded::resolve_single_ref(const llvm::Instruction *I, const llvm::Value *V, const CFG &in, std::vector<std::optional<RefMap1>> &maps, CFG::Node &node, NodeRef ref) {
    using Key = Translations::Key;
    using Map = RefMap1;
    enum ValueKind {
        INST, ARG, OTHER, UNKNOWN
    } kind;
    if (llvm::isa<llvm::Instruction>(V)) {
        kind = INST;
    } else if (llvm::isa<llvm::Argument>(V)) {
        kind = ARG;
    } else if (llvm::isa<llvm::Constant>(V) || llvm::isa<llvm::BasicBlock>(V) || llvm::isa<llvm::MetadataAsValue>(V)) {
        kind = OTHER;
    } else {
        kind = UNKNOWN;
    }
    
    switch (kind) {
        case INST:
        case ARG: {
            std::vector<Key> sources;
            in.translations.lookup(Key {node.id->func, V}, std::back_inserter(sources));
            const Map& map = *maps.at(ref);
            for (const Key& source : sources) {
                const auto it = map.find(source);
                if (kind == INST) {
                    // TODO: why make this exception for phi nodes?
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
                
                // DEBUG {=
                {
                    if (kind == ARG && it == map.end() && !node.id->func.empty() && !llvm::isa<llvm::Constant>(source.V)) {
                        std::cerr << __FUNCTION__ << ": failed to bind inlined argument\n";
                        llvm::errs() << "argument: " << *V << "\n";
                        llvm::errs() << "referenced by instruction: " << *I << "\n";
                        llvm::errs() << "in function: " << I->getFunction()->getName() << "\n";
                        llvm::errs() << "source: " << source << "\n";
                        llvm::errs() << "similar entries:\n";
                        using output::operator<<;
                        for (const auto& p : map) {
                            if (p.first.id == source.id) {
                                llvm::errs() << p.first << "," << p.second << "\n";
                            }
                            if (p.first.V == source.V) {
                                llvm::errs() << p.first << "," << p.second << "\n";
                            }
                        }
                        std::abort();
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


void CFG_Expanded::resolve_refs(const CFG& in) {
    /* Approach
     * We want to bind all llvm::Argument's and llvm::Instruction's. We should leave other kinds of llvm::Value's alone.
     */
    
    /*
     * Optimiaztion idea: remove from map once all successors have been processed
     */
    
    using Translations = CFG_Unrolled::Translations;
    using Key = Translations::Key;
    using Map = RefMap1; // TODO: remove this unnecessary definition
    
    
/* Idea: use a string-table-like approach. Rather than the map's values being a NodeRefSet, it is an index into a table of NodeRefSets.
 */
    std::vector<std::optional<Map>> maps {size()};
    
    for (NodeRef i = 0; const auto ref : reverse_postorder()) {
        assert(i == ref);
        ++i;
    }

    NodeRefSet done;
    for (const NodeRef ref : reverse_postorder()) {
        done.insert(ref);

        // merge incoming
        Map& map = *(maps[ref] = Map {size()});
        for (const NodeRef pred : po.rev.at(ref)) {
            auto& a = maps.at(pred);

            // check this predecessor will be done (all successors processed)
            const bool pred_done = util::subset(po.fwd.at(pred), done);
            if (pred_done) {
                if (map.empty()) {
                    map = std::move(*a);
                } else {
                    map.merge(*a);
                }
                a = std::nullopt;
            } else if (map.empty()) {
                map = *a;
            } else {
                for (const auto& p : *a) {
                    map[p.first].insert(p.second.begin(), p.second.end());
                }
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

