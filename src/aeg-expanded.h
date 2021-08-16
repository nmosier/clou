#pragma once

#include <vector>

#include "lcm.h"
#include "aeg-po2.h"
#include "binrel.h"

class AEGPO_Expanded {
public:
   using NodeRef = AEGPO2::NodeRef;
   
   struct Node {
      using Variant = AEGPO2::Node::Variant;
      Variant v;
   };
   
   using Rel = binrel<NodeRef>;
   Rel po;

   explicit AEGPO_Expanded(const AEGPO2& po, unsigned num_specs = 2):
      num_specs(num_specs) {
      construct(po);
   }

   NodeRef entry;
   NodeRef exit;

   std::size_t size() const { return nodes.size(); }
      
private:
   const unsigned num_specs;
   std::vector<Node> nodes;

   using NodeMap = std::unordered_map<NodeRef, NodeRef>;
   void construct(const AEGPO2& in);
   void construct_rec(const AEGPO2& in, NodeRef in_src, NodeRef in_dst, NodeRef src,
                      unsigned spec_depth, NodeMap& map);
   
   // TODO: This should be merged to shared code between all aeg-po stages.
   NodeRef add_node(const Node& node) {
      const NodeRef res = nodes.size();
      nodes.push_back(node);
      po.add_node(res);
      return res;
   }
};
