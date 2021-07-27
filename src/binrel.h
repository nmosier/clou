#pragma once

template <typename T, typename Hash = std::hash<T>>
class binrel {
public:
   using Set = std::unordered_set<T, Hash>;
   using Map = std::unordered_map<T, Set, Hash>;
   Map fwd;
   Map rev;

   void insert(const T& src, const T& dst) {
      fwd[src].insert(dst);
      rev[dst].insert(src);
   }

   void add_node(const T& node) {
      fwd[node];
      rev[node];
   }
      

   void erase(const T& src, const T& dst) {
      fwd[src].erase(dst);
      rev[dst].erase(src);
   }

private:
};
