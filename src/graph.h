#pragma once

#include <unordered_map>
#include <unordered_set>
#include <memory>

#include "tarjan.h"
#include "lcm.h"

template <typename Node, typename Edge, typename NodeHash = std::hash<Node>,
          typename EdgeHash = std::hash<Edge>>
class Graph {
public:
   using node_type = Node;
   using edge_type = Edge;

   using ESet = std::vector<std::shared_ptr<Edge>>;
   using NEMap = std::unordered_map<Node, ESet, NodeHash>;
   using NNEMap = std::unordered_map<Node, NEMap, NodeHash>;

   NNEMap fwd;
   NNEMap rev;

   NNEMap& operator()(Direction dir) {
      switch (dir) {
      case Direction::IN: return rev;
      case Direction::OUT: return fwd;
      default: std::abort();
      }
   }

   const NNEMap& operator()(Direction dir) const {
      return const_cast<NNEMap&>(const_cast<Graph&>(*this)(dir));
   }

   void insert(const Node& src, const Node& dst, const Edge& edge) {
      std::shared_ptr<Edge> edgeptr = std::make_shared<Edge>(edge);
      fwd[src][dst].push_back(edgeptr);
      rev[dst][src].push_back(edgeptr);
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

   template <typename Function>
   void for_each_edge(Function f) const {
      for (const auto& srcp : fwd) {
         for (const auto& dstp : srcp.second) {
            for (const auto& edge : dstp.second) {
               f(srcp.first, dstp.first, *edge);
            }
         }
      }
   }

   template <typename Function>
   void for_each_edge(Function f) {
      for (auto& srcp : fwd) {
         for (auto& dstp : srcp.second) {
            for (auto& edge : dstp.second) {
               f(srcp.first, dstp.first, *edge);
            }
         }
      }
   }
    
   /* How to convert strongly connected components into minimal cycles?
    *
    */

   using Path = std::vector<Node>;
   struct Cycle {
      Path nodes;
      std::vector<std::vector<Edge>> edges;
   };

   template <typename OutputIt, typename Pred>
   void cycles(OutputIt out, Pred pred) const {
      /* For now, just use simple algorithm. Use Johnson's Algorithm 
       * (https://www.cs.tufts.edu/comp/150GA/homeworks/hw1/Johnson%2075.PDF)
       * if runtime becomes an issue.
       * This current algorithm proceeds as follows:
       * - Generate simple subgraph just including allowed edges.
       * - For each node, do a BFS back to self. 
       */

      // generate simple subgraph
      std::unordered_map<Node, std::unordered_map<Node, std::vector<Edge>>> subgraph;
      std::unordered_set<Node> srcnodes, dstnodes, subnodes;
      for (const auto& p1 : fwd) {
         const Node& src = p1.first;
         for (const auto& p2 : p1.second) {
            const Node& dst = p2.first;
            for (const auto& ep : p2.second) {
               const Edge& e = *ep;
               if (pred(e)) {
                  subgraph[src][dst].push_back(e);
                  srcnodes.insert(src);
                  dstnodes.insert(dst);
               }
            }
         }
      }
      std::copy_if(srcnodes.begin(), srcnodes.end(), std::inserter(subnodes, subnodes.end()),
                   [&] (const Node& node) -> bool {
                      return dstnodes.find(node) != dstnodes.end();
                   });

#if 0
      // get cycles
      Path path;
      std::function<void(const Node&)> cycles_rec;
      cycles_rec = [&] (const Node& node) {
         path.push_back(node);
         
         // check if cycle
         if (path.size() > 1 && path.back() == node) {
            Cycle cycle;
            for (auto it1 = path.begin(), it2 = std::next(it1); it2 != path.end(); ++it1, ++it2) {
               cycle.nodes.push_back(*it1);
               cycle.edges.push_back(subgraph.at(*it1).at(*it2));
            }
            *out++ = cycle;
         } else {
            for (const auto& dst_pair : subgraph.at(node)) {
               cycles_rec(dst_pair.first);
            }
         }
         
         path.pop_back();
      };

      for (const Node& node : subnodes) {
         cycles_rec(node);
      }
#else
      // generate adjacency list
      using Tarjan = tarjan<Node, NodeHash>;
      typename Tarjan::DAG g;
      for (const auto& p1 : subgraph) {
         for (const auto& p2 : p1.second) {
            g[p1.first].insert(p2.first);
         }
      }

      // invoke tarjan
      std::vector<typename Tarjan::Cycle> cycles;
      Tarjan(g, std::back_inserter(cycles));

      for (const typename Tarjan::Cycle& cycle_ : cycles) {
         Cycle c;
         std::copy(cycle_.begin(), cycle_.end(), std::back_inserter(c.nodes));
         c.nodes.push_back(c.nodes.front());
         for (auto it1 = c.nodes.begin(), it2 = std::next(it1);
              it2 != c.nodes.end();
              ++it1, ++it2) {
            c.edges.push_back(subgraph.at(*it1).at(*it2));
         }
         c.nodes.pop_back();
         *out++ = std::move(c);
      }
#endif
   }

private:
};
