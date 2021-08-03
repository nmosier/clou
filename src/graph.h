#pragma once

#include <unordered_map>
#include <unordered_set>

#include "util.h"

template <typename Node, typename Edge, typename NodeHash = std::hash<Node>,
          typename EdgeHash = std::hash<Edge>>
class Graph {
public:
   using node_type = Node;
   using edge_type = Edge;

   using ESet = std::unordered_set<Edge, EdgeHash>;
   using NEMap = std::unordered_map<Node, ESet, NodeHash>;
   using NNEMap = std::unordered_map<Node, NEMap, NodeHash>;

   NNEMap fwd;
   NNEMap rev;

   void insert(const Node& src, const Node& dst, const Edge& edge) {
      fwd[src].emplace(dst, edge);
      rev[dst].emplace(src, edge);
   }

   template <typename InputIt>
   void insert(InputIt src_begin, InputIt src_end, const Node& dst, const Edge& edge) {
      for (auto src_it = src_begin; src_it != src_end; ++src_it) {
         insert(*src_it, dst, edge);
      }
   }

   template <typename InputIt>
   void insert(const Node& src, InputIt dst_begin, InputIt dst_end, const Edge& edge) {
      for (auto dst_it = dst_begin; dst_it != dst_end; ++dst_it) {
         insert(src, *dst_it, edge);
      }
   }

   template <typename InputIt1, typename InputIt2>
   void insert(InputIt1 src_begin, InputIt1 src_end, InputIt2 dst_begin, InputIt2 dst_end,
               const Edge& edge) {
      for (auto src_it = src_begin; src_it != src_end; ++src_it) {
         for (auto dst_it = dst_begin; dst_it != dst_end; ++dst_it) {
            insert(*src_it, *dst_it, edge);
         }
      }
   }

   void add_node(const Node& node) {
      fwd[node];
      rev[node];
   }

   void erase(const Node& src, const Node& dst, const Edge& edge) {
      fwd[src][dst].erase(edge);
      rev[dst][src].erase(edge);
   }

   void erase(const Node& src, const Node& dst) {
      fwd[src].erase(dst);
      rev[dst].erase(src);
   }

   void erase(const Node& node) {
      const auto f = [&] (NNEMap& rel1, NNEMap& rel2) {
         auto it = rel1.find(node);
         if (it != rel1.end()) {
            for (const Node& other : it->second) {
               rel2.at(other).erase(node);
            }
            rel1.erase(it);
         }
      };

      f(fwd, rev);
      f(rev, fwd);
   }

private:
};
