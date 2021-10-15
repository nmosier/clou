#pragma once

#include <iostream>
#include <utility>
#include <unordered_set>
#include <optional>
#include <vector>
#include <variant>

/// Print std::optional
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::optional<T>& x) {
    if (x) {
        os << *x;
    } else {
        os << "(nullopt)";
    }
    return os;
}

/// Print std::pair
template <typename T, typename U>
std::ostream& operator<<(std::ostream& os, const std::pair<T, U>& pair) {
    return os << "(" << pair.first << ", " << pair.second << ")";
}

/// Print std::unordered_set
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::unordered_set<T>& set) {
    os << "{";
    for (auto it = set.begin(); it != set.end(); ++it) {
        if (it != set.begin()) {
            os << ", ";
        }
        os << *it;
    }
    os << "}";
    return os;
}

/// Print tuple
template <typename... Ts>
std::ostream& operator<<(std::ostream& os, const std::tuple<Ts...>& tuple) {
    os << "(";
    TuplePrinter<Ts...>().template print<0>(os, tuple);
    os << ")";
    return os;
}

/// Print vector
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
    os << "{";
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (it != v.begin()) {
            os << ", ";
        }
        os << *it;
    }
    os << "}";
    return os;
}

/// Print variant
template <typename... Ts>
std::ostream& operator<<(std::ostream& os, const std::variant<Ts...>& v) {
    std::visit([&] (const auto& x) {
        os << x;
    }, v);
    return os;
}
