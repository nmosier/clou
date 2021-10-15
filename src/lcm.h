#pragma once

#include <set>
#include <tuple>
#include <unordered_set>
#include <unordered_map>
#include <iostream>

#include <llvm/IR/Instruction.h>

struct Entry {};
struct Exit {};

inline bool operator==(Entry, Entry) { return true; }
inline bool operator==(Exit, Exit) { return true; }

inline std::ostream& operator<<(std::ostream& os, Entry) {
    return os << "<ENTRY>";
}

inline std::ostream& operator<<(std::ostream& os, Exit) {
    return os << "<EXIT>";
}

enum class Option {
    MUST, MAY, NO
};

enum class Direction {
    IN, OUT
};

inline std::ostream& operator<<(std::ostream& os, Direction dir) {
    switch (dir) {
        case Direction::IN: return os << "IN";
        case Direction::OUT: return os << "OUT";
        default: return os << "(invalid)";
    }
}

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
