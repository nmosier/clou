#include <deque>

#include "aeg-expanded.h"

void AEGPO_Expanded::construct(const AEGPO2& in) {
   // create entry
   NodeRef in_src = in.entry;
   NodeRef src = add_node(Node {in.lookup(in_src).v});
   NodeMap map {{in_src, src}};
   for (NodeRef in_dst : in.po.fwd.at(in.entry)) {
      construct_rec(in, in_src, in_dst, src, 0, map);
   }
}

/* NOTE: spec_depth is the depth of in_src.
 *
 */
void AEGPO_Expanded::construct_rec(const AEGPO2& in, NodeRef in_src, NodeRef in_dst, NodeRef src,
                                   unsigned spec_depth, NodeMap& map) {
   NodeRef dst;

   /* check whether to merge or duplicate node */
   ++spec_depth;
   if (spec_depth < num_specs) {
      /* create private node */
      dst = add_node(Node {in.lookup(in_dst).v});
   } else {
      /* merge with or create public node */
      const auto it = map.find(in_dst);
      if (it == map.end()) {
         // create
         dst = add_node(Node {in.lookup(in_dst).v});
         map.emplace(in_dst, dst);
      } else {
         dst = it->second;
      }
   }
   
   po.insert(src, dst);

   const auto& succs = in.po.fwd.at(in_dst);
   if (succs.size() > 1) {
      /* speculation depth reset to 0 */
      spec_depth = 0;
   }

   const NodeRef next_in_src = in_dst;
   for (NodeRef next_in_dst : succs) {
      construct_rec(in, next_in_src, next_in_dst, dst, spec_depth, map);
   }
}
