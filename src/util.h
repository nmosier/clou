#pragma once

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <sstream>
#include <type_traits>
#include <numeric>

#include <llvm/IR/Function.h>
#include <llvm/Support/Format.h>

#include "binrel.h"

extern const char *prog;

template <typename... Args>
void log(const char *fmt, Args&&... args) {
   llvm::errs() << llvm::format(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[noreturn]] void error(const char *fmt, Args&&... args) {
   log(fmt, std::forward<Args>(args)...);
   exit(1);
}

extern unsigned verbose;
template <typename... Args>
void log(unsigned verb, const char *fmt, Args&&... args) {
   if (verbose >= verb) {
       log(fmt, std::forward<Args>(args)...);
   }
}

inline llvm::raw_ostream& logv(unsigned verb) {
   if (verbose >= verb) {
      return llvm::errs();
   } else {
      static llvm::raw_null_ostream null_os;
      return null_os;
   }
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


namespace util {
   template <typename T>
   struct identity {
      constexpr T&& operator()(T&& t) const noexcept { return std::forward<T>(t); }
      constexpr const T& operator()(const T& t) const noexcept { return t; }
   };

   struct Void {};
}

namespace util {
   template <typename T, typename InputIt, typename UnaryPredicate>
   T all_of(InputIt begin, InputIt end, UnaryPredicate p, T true_val) {
      return std::transform_reduce(begin, end, true_val, [] (const T& a, const T& b) -> T {
         return a && b;
      }, p);
   }

   // TODO: Rewrite using transform_reduce.
   template <typename T, typename InputIt, typename UnaryPredicate>
   T one_of(InputIt begin, InputIt end, UnaryPredicate p, T true_val, T false_val) {
      T disj = false_val;
      for (auto it1 = begin; it1 != end; ++it1) {
         const T conj = all_of(begin, end, [&] (const auto& val) {
            auto pred = p(val);
            if (&val != &*it1) {
               pred = !pred;
            }
            return pred;
         }, true_val);
         disj = disj || conj;
         // disj |= conj;
      }
      return disj;
   }

   template <typename T, typename InputIt, typename UnaryPredicate>
   T any_of(InputIt begin, InputIt end, UnaryPredicate p, T false_val) {
      return std::transform_reduce(begin, end, false_val,
                                   [] (const T& a, const T& b) -> T {
                                      return a || b;
                                   }, p);
   }
   
   template <typename T>
   class RangeIterator {
   public:
      const T& operator*() const { return val; }
      const T *operator->() const { return &val; }
      RangeIterator& operator++() { ++val; return *this; }
      RangeIterator& operator++(int) { ++val; return *this; }
      bool operator==(const RangeIterator& other) const { return val == other.val; }
      bool operator!=(const RangeIterator& other) const { return !(*this == other); }
      RangeIterator(const T& val): val(val) {}
   private:
      T val;
   };

   template <typename T>
   using RangeContainer = llvm::iterator_range<RangeIterator<T>>;

   template <typename T>
   std::string to_string(const T& x) {
      std::stringstream ss;
      ss << x;
      return ss.str();
   }

   template <typename InputIt>
   std::string to_string(InputIt begin, InputIt end, const std::string& sep = " ") {
      std::stringstream ss;
      for (InputIt it = begin; it != end; ++it) {
         if (it != begin) {
            ss << sep;
         }
         ss << *it;
      }
      return ss.str();
   }

#if 0
    /* Generalized for-each */
   template <size_t N, typename Container, typename UnaryOp, typename... Args>
   void for_each(const Container& c, UnaryFunction f) {
      if constexpr (N == 0) {
         f(c); 
      } else {
         std::for_each(c.begin(), c.end(), [f] (const auto& e) {
            for_each<N-1>(e, f);
         });
      }
   }
#endif

   template <typename T>
   struct creator {
      template <typename... Args>
      T operator()(Args&&... args) const { return T {std::forward<Args>(args)...}; }
   };

   template <typename T>
   struct logical_or {
      auto operator()(const T& a, const T& b) const { return a || b; }
   };

   template <typename T>
   struct logical_and {
      auto operator()(const T& a, const T& b) const { return a && b; }
   };


   template <typename T>
   T replace(T& oldval, const T& newval) {
      const T res = oldval;
      oldval = newval;
      return res;
   }

   template <typename T>
   T replace(T& oldval, T&& newval) {
      const T res = oldval;
      oldval = newval;
      return res;
   }

}
