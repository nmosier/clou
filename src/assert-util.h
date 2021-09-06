#pragma once

#include <cassert>
#include <iostream>

#define assert_eq(lhs, rhs) assert_eq_(lhs, rhs, __FILE__, __LINE__, #lhs, #rhs)

template <typename T, typename U>
void assert_eq_(const T& lhs, const U& rhs, const char *file, unsigned line, const char *lhs_str, const char *rhs_str) {
    if (!(lhs == rhs)) {
        std::cerr << "Assertion failed (eq): " << file << ":" << line << ": " << lhs_str << " == " << rhs_str << ": " << lhs << " != " << rhs << "\n";
        std::abort();
    }
}
