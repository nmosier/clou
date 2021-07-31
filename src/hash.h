#pragma once

inline void hash_combine_ordered(std::size_t& seed) {}

template <typename T, typename... Args>
inline void hash_combine_ordered(std::size_t& seed, const T& v, Args&&... tail) {
   std::hash<T> hasher;
   seed ^= hasher(v) + 0xa9e3779b9 + (seed<<6) + (seed>>2);
   hash_combine_ordered(seed, std::forward<Args>(tail)...);
}

template <typename... Args>
std::size_t hash_ordered_tuple(Args&&... args) {
   std::size_t seed = 0;
   hash_combine_ordered(seed, std::forward<Args>(args)...);
   return seed;
}

template <typename InputIt>
std::size_t hash_ordered_sequence(InputIt begin, InputIt end) {
   std::size_t seed = 0;
   for (auto it = begin; it != end; ++it) {
      hash_combine_ordered(seed, *it);
   }
   return seed;
}

inline void hash_combine_unordered(std::size_t& seed) {}

template <typename T, typename... Args>
inline void hash_combine_unordered(std::size_t& seed, const T& v, Args&&... tail) {
   std::hash<T> hasher;
   seed ^= hasher(v);
   hash_combine_unordered(seed, std::forward<Args>(tail)...);
}

template <typename... Args>
std::size_t hash_unordered_tuple(Args&&... args) {
   std::size_t seed = 0;
   hash_combine_unordered(seed, std::forward<Args>(args)...);
   return seed;
}

template <typename InputIt>
std::size_t hash_unordered_sequence(InputIt begin, InputIt end) {
   std::size_t seed = 0;
   for (auto it = begin; it != end; ++it) {
      hash_combine_unordered(seed, *it);
   }
   return seed;
}

namespace util {
   template <typename T, typename... Args>
   auto do_hash(const T& val, Args&&... args) {
      return std::hash<T>()(val);
   }
}

namespace std {

   template <typename... Args>
   struct hash<unordered_set<Args...>> {
      size_t operator()(const unordered_set<Args...>& set) const {
         return hash_unordered_sequence(set.begin(), set.end());
      }
   };

   template <typename... Args>
   struct hash<vector<Args...>> {
      size_t operator()(const vector<Args...>& vec) const {
         return hash_ordered_sequence(vec.begin(), vec.end());
      }
   };

   template <typename First, typename Second>
   struct hash<pair<First, Second>> {
      size_t operator()(const pair<First, Second>& pair) const {
         return hash_ordered_tuple(pair.first, pair.second);
      }
   };
}
