#include <iostream>

#include "kosaraju.h"

int main(int argc, char *argv[]) {
    using Kosaraju_ = Kosaraju<int>;
    Kosaraju_::AdjacencyMap edges;
    Kosaraju_::NodeList nodes;
    {
        int src, dst;
        while (std::cin >> src >> dst) {
            nodes.insert(src);
            nodes.insert(dst);
            edges[src].insert(dst);
        }
    }
    
    std::vector<std::unordered_set<int>> sccs;
    Kosaraju_(nodes, edges)(std::back_inserter(sccs));
    
    for (const auto& scc : sccs) {
        for (const auto node : scc) {
            std::cout << node << " ";
        }
        std::cout << "\n";
    }
}
