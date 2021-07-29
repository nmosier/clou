#pragma once

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <sstream>

#include <llvm/IR/Function.h>

#include "binrel.h"

extern const char *prog;

template <typename... Args>
void log(const char *fmt, Args&&... args) {
   fprintf(stderr, fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[noreturn]] void error(const char *fmt, Args&&... args) {
   log(fmt, std::forward<Args>(args)...);
   exit(1);
}

using BinaryInstRel = std::unordered_map<const llvm::Instruction *,
                                         std::unordered_set<const llvm::Instruction *>
                                         >;



void predecessor_map(const llvm::Function& F, BinaryInstRel& preds);
void successor_map(const llvm::Function& F, BinaryInstRel& succs);

namespace util {

   template <typename It>
   class Range {
   public:
      constexpr Range(It begin, It end): begin_(begin), end_(end) {}
      constexpr It begin() const { return begin_; }
      constexpr It end() const { return end_; }
   private:
      const It begin_;
      const It end_;
   };

   template <typename It>
   constexpr Range<It> make_range(It begin, It end) { return Range {begin, end}; }

}

inline const char *getenvs(const char *name) {
   if (char *value = getenv(name)) {
      return value;
   } else {
      error("key '%s' does not exist", name);
   }
}

inline const char *getenvs(const char *name, const char *dfl) {
   if (const char *value = getenv(name)) {
      dfl = value;
   }
   return dfl;
}

template <typename Int>
Int getenvi(const char *name) {
   const char *s = getenvs(name);
   std::stringstream ss {s};
   Int res;
   ss >> res;
   return res;
}

template <typename Int>
Int getenvi(const char *name, Int dfl) {
   if (const char *s = getenv(name)) {
      std::stringstream ss {s};
      ss >> dfl;
   }
   return dfl;
}

inline bool getenvb(const char *name) {
   return getenvs(name) != nullptr;
}


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


using CFG = binrel<const llvm::Instruction *>;



void get_cfg(const llvm::Function& F, CFG& cfg);


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
}


template <typename... Args>
std::string format(const std::string& fmt, Args&&... args) {
   char *s;
   if (asprintf(&s, fmt.c_str(), std::forward<Args>(args)...) < 0) {
      throw std::system_error(errno, std::generic_category(), "asprintf");
   }
   std::string res {s};
   free(s);
   return res;
}


inline std::string format_graph_path(const std::string& fmt, const llvm::Function& F) {
   return format(fmtcheck(fmt.c_str(), "%s.dot"), F.getName().str().c_str());
}
