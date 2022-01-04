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

    template <typename OutputIt>
    OutputIt operator()(OutputIt out) {
        for (T node : nodes) {
            visit(node);
        }

        for (auto it = L.rbegin(); it != L.rend(); ++it) {
            const T node = *it;
            assign(node, node);
        }

        std::unordered_map<T, SCC> sccs2;
        for (const auto& pair : sccs) {
            sccs2[pair.second].insert(pair.first);
        }
        
        return std::transform(sccs2.begin(), sccs2.end(), out,
                              [] (const auto& pair) {
            return pair.second;
        });
    }
    
private:
    NodeList nodes;
    AdjacencyMap edges;
    AdjacencyMap redges;
    std::vector<T> L;
    std::unordered_set<T> visited;
    std::unordered_map<T, T> sccs;
    
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
        if (!sccs.contains(node)) {
            sccs.emplace(node, root);
            for (T pred : redges[node]) {
                assign(pred, root);
            }
        }
    }
};
