#pragma once

#include <unordered_map>
#include <unordered_set>

#include "util.h"

/* The graph needs to provide directed and undirected edges. How to handle?
 * Do we need to distinguish between bidirectional and undirectional edges?
 * Perhaps table this for now.
 * We are looking for transiently executed branches that leak information. 
 */
template <typename T, typename U> 
class Graph {
public:
   using Node = T;
   using Edge = U;
   using EdgePtr = std::shared_ptr<Edge *>;
   using Nodes = std::unordered_set<Node>;

   bool emplace(const Node& src, const Node& dst, const Edge& edge) {
      EdgePtr edgeptr = std::make_shared<Edge>(edge);
      const auto fwd_res = fwd[src][dst].insert(edgeptr);
      const auto rev_res = rev[dst][src].insert(edgeptr);
      nodes.insert(src);
      nodes.insert(dst);
      assert(fwd_res.second == rev_res.second);
      return fwd_res.second;
   }

   bool erase(const Node& src, const Node& dst, const Edge& edge) {
      const auto fwd_res = erase_edges(fwd[src][dst], edge);
      const auto rev_res = erase_edges(rev[dst][src], edge);
      assert(fwd_res == rev_res);
      return fwd_res;
   }

   /* Node Iterator */
   using NodeIterator_All = typename Nodes::const_iterator;
   NodeIterator_All nodes_all_begin() const { return nodes.begin(); }
   NodeIterator_All nodes_all_end() const { return nodes.end(); }
   using NodeRange_All = Range<NodeIterator_All>;
   NodeRange_All nodes_all() const { return make_range(nodes_all_begin(), nodes_all_end()); }

   class NodeIterator_Neighbors;
   using NodeRange_Neighbors = Range<NodeIterator_Neighbors>;
   NodeIterator_Neighbors nodes_srcs_begin(const Node& dst) const {
      return NodeIterator_Neighbors {rev.find(dst)};
   }
   NodeIterator_Neighbors nodes_srcs_end(const Node& dst) const {
      return NodeIterator_Neighbors {rev.find(dst)};
   }
   NodeRange_Neighbors nodes_srcs(const Node& dst) const {
      return make_range(nodes_srcs_begin(dst), nodes_srcs_end(dst));
   }
   NodeIterator_Neighbors nodes_dsts_begin(const Node& src) const {
      return NodeIterator_Neighbors {fwd.find(src)};
   }
   NodeIterator_Neighbors nodes_dsts_end(const Node& src) const {
      return NodeIterator_Neighbors {fwd.find(src)};
   }
   NodeRange_Neighbors nodes_dsts(const Node& src) const {
      return make_range(nodes_dsts_begin(src), nodes_dsts_end(src));
   }

private:
   using EdgeList = std::vector<EdgePtr>;
   using DstMap = std::unordered_map<Node, EdgeList>;
   using Map = std::unordered_map<Node, DstMap>;
   Map fwd;
   Map rev;
   Nodes nodes;
   
   static bool erase_edge(EdgeList& edges, const Edge& edge) {
      for (auto it = edges.begin(); it != edges.end(); ++it) {
         if (**it == edge) {
            edges.erase(it);
            return true;
         }
      }
      return false;
   }
};

template <typename T, typename U>
class Graph<T,U>::NodeIterator_Neighbors: public std::iterator<std::input_iterator_tag, T> {
public:
   using It = typename DstMap::const_iterator;
   NodeIterator_Neighbors(It it): it(it) {}

   NodeIterator_Neighbors operator++() { ++it; }
   bool operator==(const NodeIterator_Neighbors& other) const { return it == other.it; }
   bool operator!=(const NodeIterator_Neighbors& other) const { return !(*this == other); }
   const T& operator*() const { return it->first; }
   
private:
   It it;
};
