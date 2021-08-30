#pragma once

#include <vector>
#include <unordered_map>

#include "lcm.h"
#include "aeg-po2.h"
#include "binrel.h"
#include "config.h"

class AEGPO_Expanded: public AEGPO {
public:
   explicit AEGPO_Expanded(unsigned num_specs): AEGPO(num_specs) {}

   void construct(const AEGPO& in);

private:
   using NodeMap = std::unordered_map<NodeRef, NodeRef>;

   struct Task {
      NodeRef in_src;
      NodeRef in_dst;
      NodeRef src;
      unsigned spec_depth;
   };
   template <typename OutputIt>
   void construct_rec(const AEGPO& in, NodeRef in_src, NodeRef in_dst,
                      NodeRef src, unsigned spec_depth, NodeMap& map, OutputIt out);

};
