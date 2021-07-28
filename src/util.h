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

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
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
   struct hash<std::unordered_set<Args...>> {
      size_t operator()(const std::unordered_set<Args...>& set) const {
         size_t res = 0;
         for (const auto& x : set) {
            res ^= set.hash_function()(x);
         }
         return res;
      }
   };
   
}
