#pragma once

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

/** Get strongly connected components from  input adjacency list.
 *
 */

template <typename T>
class Kosaraju {
public:
    using Type = T;
    using SCC = std::unordered_set<T>;
    using AdjacencyMap = std::unordered_map<T, std::unordered_set<T>>;
    using NodeList = std::unordered_set<T>;
    
    Kosaraju(const NodeList& nodes, const AdjacencyMap& edges): nodes(nodes), edges(edges) {
        reverse(edges, redges);
    }
    
    Kosaraju(const AdjacencyMap& edges): edges(edges) {
        for (const auto& pair : edges) {
            nodes.insert(pair.first);
            for (const auto& node : pair.second) {
                nodes.insert(node);
            }
        }
        reverse(edges, redges);
    }

    template <typename OutputIt>
    OutputIt operator()(OutputIt out) {
        for (T node : nodes) {
            visit(node);
        }

        for (auto it = L.rbegin(); it != L.rend(); ++it) {
            const T node = *it;
            assign(node, node);
        }
        
        for (const auto& pair : scc_assignments) {
            scc_map[pair.second].insert(pair.first);
        }
        
        return std::transform(scc_map.begin(), scc_map.end(), out,
                              [] (const auto& pair) {
            return pair.second;
        });
    }
    
    T get_root(T node) const {
        return scc_assignments.at(node);
    }
    
    const SCC& get_scc(T node) const {
        return scc_map.at(get_root(node));
    }
    
private:
    NodeList nodes;
    AdjacencyMap edges;
    AdjacencyMap redges;
    std::vector<T> L;
    std::unordered_set<T> visited;
    std::unordered_map<T, T> scc_assignments;
    std::unordered_map<T, SCC> scc_map;
    
    static void reverse(const AdjacencyMap& in, AdjacencyMap& out) {
        for (const auto& in_pair : in) {
            const T src = in_pair.first;
            for (const T dst : in_pair.second) {
                out[dst].insert(src);
            }
        }
    }
    
    void visit(T node) {
        if (visited.insert(node).second) {
            for (T succ : edges[node]) {
                visit(succ);
            }
            L.push_back(node);
        }
    }
    
    void assign(T node, T root) {
        if (!scc_assignments.contains(node)) {
            scc_assignments.emplace(node, root);
            for (T pred : redges[node]) {
                assign(pred, root);
            }
        }
    }
};
