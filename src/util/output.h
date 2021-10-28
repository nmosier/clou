#pragma once

#include <iostream>
#include <utility>
#include <unordered_set>
#include <optional>
#include <vector>
#include <variant>

namespace output {

template <typename OutputStream, typename InputIt, typename Transform>
OutputStream& range(OutputStream& os, InputIt begin, InputIt end, const std::string& sep, Transform transform) {
    using ::operator<<;
    os << "{";
    for (auto it = begin; it != end; ++it) {
        if (it != begin) {
            os << sep;
        }
        os << transform(*it);
    }
    os << "}";
    return os;
}

template <typename OutputStream, typename Container, typename Transform>
OutputStream& container(OutputStream& os, const Container& container, const std::string& sep, Transform transform) {
    return range(os, container.begin(), container.end(), sep, transform);
}

#define declare_container(name) \
template <typename OutputStream, typename... Args> \
OutputStream& operator<<(OutputStream& os, const name<Args...>& c) { \
return container(os, c, ", ", [] (const auto& x) { return x; }); \
}

declare_container(std::vector)
declare_container(std::unordered_set)

}



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

/// Print std::unordered_map
template <typename T, typename U>
std::ostream& operator<<(std::ostream& os, const std::unordered_map<T, U>& map) {
    os << "{";
    for (auto it = map.begin(); it != map.end(); ++it) {
        if (it != map.begin()) {
            os << ", ";
        }
        os << *it;
    }
    os << "}";
    return os;
}


namespace detail {
template <typename... Ts>
struct TuplePrinter {
    using Tuple = std::tuple<Ts...>;
    friend std::ostream& operator<<(std::ostream& os, const Tuple& tuple);
    
    template <std::size_t i>
    void print(std::ostream& os, const Tuple& tuple) const {
        using ::operator<<;
        using output::operator<<;   
        if constexpr (i < sizeof...(Ts)) {
            if constexpr (i > 0) {
                os << ", ";
            }
            os << std::get<i>(tuple);
            print<i+1>(os, tuple);
        }
    }
};
}

/// Print tuple
template <typename... Ts>
std::ostream& operator<<(std::ostream& os, const std::tuple<Ts...>& tuple) {
    os << "(";
    detail::TuplePrinter<Ts...>().template print<0>(os, tuple);
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
