#include <deque>

#include "aeg-expanded.h"

void AEGPO_Expanded::construct(const AEGPO2& in) {
   // create entry
   NodeRef in_src = in.entry;
   NodeRef src = add_node(in.lookup(in_src));
   entry = src;
   NodeMap map {{in_src, src}};

   std::deque<Task> queue;
   for (NodeRef in_dst : in.po.fwd.at(in_src)) {
      queue.push_back({in_src, in_dst, src, 0});
   }
   while (!queue.empty()) {
      const Task& task = queue.back();
      construct_rec(in, task.in_src, task.in_dst, task.src, task.spec_depth, map,
                    std::front_inserter(queue));
      queue.pop_back();
   }
}

/* NOTE: spec_depth is the depth of in_src.
 *
 */
template <typename OutputIt>
void AEGPO_Expanded::construct_rec(const AEGPO2& in, NodeRef in_src, NodeRef in_dst,
                                   NodeRef src, unsigned spec_depth, NodeMap& map, OutputIt out) {
   NodeRef dst;
   bool newnode;
   
   /* check whether to merge or duplicate node */
   ++spec_depth;
   if (spec_depth < num_specs) {
      /* create private node */
      dst = add_node(in.lookup(in_dst));
      newnode = true;
      llvm::errs() << "private " << dst << "\n";
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
      llvm::errs() << "public " << dst << "\n";
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
   }
}
