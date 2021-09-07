#include <deque>

#include "aeg-expanded.h"
#include "aeg-unrolled.h"
#include "util.h"

void AEGPO_Expanded::construct(const AEGPO_Unrolled& in) {
   // create entry
   NodeRef in_src = in.entry;
   NodeRef src = add_node(in.lookup(in_src));
   entry = src;
   NodeMap map {{in_src, src}};

   std::deque<Task> queue;
   for (NodeRef in_dst : in.po.fwd.at(in_src)) {
      queue.push_back({in_src, in_dst, src, num_specs}); // since ENTRY will never be speculatively executed
   }
   while (!queue.empty()) {
      const Task& task = queue.back();
      construct_rec(in, task.in_src, task.in_dst, task.src, task.spec_depth, map,
                    std::front_inserter(queue));
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
            logv(3) << "expanded node " << pair.second.size() << " times: " << in.lookup(pair.first) << "\n";
        }
    }
    
    // remaining tasks
    resolve_refs(in);
}

/* NOTE: spec_depth is the depth of in_src.
 *
 */
template <typename OutputIt>
void AEGPO_Expanded::construct_rec(const AEGPO_Unrolled& in, NodeRef in_src, NodeRef in_dst,
                                   NodeRef src, unsigned spec_depth, NodeMap& map, OutputIt out) {
   NodeRef dst;
   bool newnode;
   
   /* check whether to merge or duplicate node */
   ++spec_depth;

   if (spec_depth <= num_specs) {
      /* create private node */
      dst = add_node(in.lookup(in_dst));
      newnode = true;
   } else {
      /* merge with or create public node */
      const auto it = map.find(in_dst);
      if (it == map.end()) {
         // create
         dst = add_node(in.lookup(in_dst));
         map.emplace(in_dst, dst);
         newnode = true;
      } else {
         dst = it->second;
         newnode = false;
      }
   }
   
   po.insert(src, dst);

   const auto& succs = in.po.fwd.at(in_dst);
   if (succs.size() > 1) {
      /* speculation depth reset to 0 */
      spec_depth = 0;
      }

   if (newnode) {
      const NodeRef next_in_src = in_dst;
      for (NodeRef next_in_dst : succs) {
         *out++ = Task {next_in_src, next_in_dst, dst, spec_depth};
      }
       expansions[in_dst].insert(dst);
   }
}

void AEGPO_Expanded::resolve_refs(const AEGPO_Unrolled& in) {
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
        if (const auto *Ip = std::get_if<const llvm::Instruction *>(&node.v)) {
            map[Key {node.id->func, *Ip}].insert(ref);
        }
        
        maps[ref] = std::move(map);
    }
    
    /* Now resolve refs */
    for (const NodeRef ref : order) {
        Node& node = lookup(ref);
        node.refs.clear();
        if (const auto *Ip = std::get_if<const llvm::Instruction *>(&node.v)) {
            const auto *I = *Ip;
            for (const llvm::Value *V : I->operand_values()) {
                enum ValueKind {
                    INST, ARG, OTHER
                } kind;
                if (llvm::dyn_cast<llvm::Instruction>(V)) {
                    kind = INST;
                } else if (llvm::dyn_cast<llvm::Argument>(V)) {
                    kind = ARG;
                } else {
                    kind = OTHER;
                }

                if (kind != OTHER) {
                    std::vector<Key> sources;
                    in.translations.lookup(Key {node.id->func, V}, std::back_inserter(sources));
                    const Map& map = maps.at(ref);
                    for (const Key& source : sources) {
                        const auto it = map.find(source);
                        if (kind == INST) {
                            assert(it != map.end());
                        }
                        if (it != map.end()) {
                            const NodeRefSet& refs = it->second;
                            node.refs[V].insert(refs.begin(), refs.end());
                        }
                    }
                }
            }
        }
    }
}
