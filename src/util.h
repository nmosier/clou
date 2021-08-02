#pragma once

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <sstream>
#include <type_traits>

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


// const llvm::Instruction * constexpr ENTRY = reinterpret_cast<const llvm::Instruction *>(0);
// const llvm::Instruction * constexpr EXIT = 

namespace util {
   // From https://en.cppreference.com/w/cpp/utility/variant/visit
   // For constructing overloaded lambdas.
   
   // helper type for the visitor #4
   template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
   // explicit deduction guide (not needed as of C++20)
   template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}


struct Empty {
   bool operator==(const Empty& other) const { return true;  }
   bool operator!=(const Empty& other) const { return false; }
};
