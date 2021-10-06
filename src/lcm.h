#pragma once

#include <set>
#include <tuple>
#include <unordered_set>
#include <unordered_map>

#include <llvm/IR/Instruction.h>

struct Entry {};
struct Exit {};

enum class Option {
    MUST, MAY, NO
};

namespace std {
   template <>
   struct hash<Entry> {
      std::size_t operator()() const { return 0; }
   };

   template <>
   struct hash<Exit> {
      std::size_t operator()() const { return 0; }
   };
}
