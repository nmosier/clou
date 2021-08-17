#pragma once

#include <vector>

#include "lcm.h"
#include "aeg-po2.h"
#include "binrel.h"

class AEGPO_Expanded: public AEGPO_Base<AEGPO_Node_Base> {
public:
   explicit AEGPO_Expanded(const AEGPO2& po, unsigned num_specs = 2):
      num_specs(num_specs) {
      construct(po);
   }

private:
   const unsigned num_specs;

   using NodeMap = std::unordered_map<NodeRef, NodeRef>;
   void construct(const AEGPO2& in);

   struct Task {
      NodeRef in_src;
      NodeRef in_dst;
      NodeRef src;
      unsigned spec_depth;
   };
   template <typename OutputIt>
   void construct_rec(const AEGPO2& in, NodeRef in_src, NodeRef in_dst,
                      NodeRef src, unsigned spec_depth, NodeMap& map, OutputIt out);

};
