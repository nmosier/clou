#pragma once

#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <vector>
#include <cassert>

template <typename Node, typename Hash = std::hash<Node>, typename Equal = std::equal_to<Node>>
class tarjan {
public:
   using NodeVec = std::vector<Node>;
   using NodeSet = std::unordered_set<Node, Hash, Equal>;
   using DAG = std::unordered_map<Node, NodeSet, Hash, Equal>;
   using Cycle = std::vector<Node>;

   template <typename OutputIt>
   tarjan(const DAG& dag, OutputIt out): dag(dag) { tarjan_(out); }
   
private:
   DAG dag;
   NodeVec nodes;
   std::unordered_map<Node, unsigned, Hash, Equal> nodes_map;
   NodeVec point_stack;
   NodeVec marked_stack;
   NodeSet marked;

   void add_node(const Node& v) {
      if (nodes_map.emplace(v, nodes.size()).second) {
         nodes.push_back(v);
      }
   }
   
   template <typename OutputIt>
   void tarjan_(OutputIt out) {
      get_nodes();

      for (const Node& s : nodes) {
         backtrack(s, s, out);
         while (!marked_stack.empty()) {
            const Node u = marked_stack.back();
            marked_stack.pop_back();
            marked.erase(u);
         }
      }
   }

   void get_nodes() {
      for (const auto& p1 : dag) {
         add_node(p1.first);
         for (const auto& p2 : p1.second) {
            add_node(p2);
         }
      }
   }

   unsigned get_index(const Node& v) const {
      return nodes_map.at(v);
   }

   template <typename OutputIt>
   bool backtrack(const Node& s, const Node& v, OutputIt& out) {
      const unsigned si = get_index(s);
      bool g;
      bool f = false;
      point_stack.push_back(v);
      marked.insert(v);
      marked_stack.push_back(v);

      auto& v_succs = dag[v];
      for (auto w_it = v_succs.begin(); w_it != v_succs.end(); ) {
         const Node& w = *w_it;
         const unsigned wi = get_index(w);
         if (wi < si) {
            w_it = v_succs.erase(w_it);
         } else {
            if (wi == si) {
               const auto it = std::find(point_stack.begin(), point_stack.end(), s);
               assert(it != point_stack.end());
               Cycle cycle;
               std::copy(it, point_stack.end(), std::back_inserter(cycle));
               *out++ = cycle;
               f = true;
            } else if (marked.find(w) == marked.end()) {
               g = backtrack(s, w, out);
               f = f || g;
            }
            ++w_it;
         }
      }

      if (f) {
         Node u;
         do {
            u = marked_stack.back();
            marked_stack.pop_back();
            marked.erase(u);
         } while (u != v);
      }

      point_stack.pop_back();

      return f;
   }
};
