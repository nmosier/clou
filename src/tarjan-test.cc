#include <iostream>
#include <vector>

#include "tarjan.h"

int main(int argc, char *argv[]) {
   using Tarjan = tarjan<unsigned>;
   Tarjan::DAG g;

   unsigned a, b;
   while (std::cin >> a >> b) {
      g[a].insert(b);
   }

   std::vector<Tarjan::Cycle> cycles;
   Tarjan(g, std::back_inserter(cycles));

   for (const auto& cycle : cycles) {
      for (unsigned v : cycle) {
         std::cout << " " << v;
      }
      std::cout << "\n";
   }
}
