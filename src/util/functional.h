#pragma once


namespace util {

template <typename T>
struct logical_or {
    auto operator()(const T& a, const T& b) const { return a || b; }
};

template <typename T>
struct logical_and {
    auto operator()(const T& a, const T& b) const { return a && b; }
};

template <typename T>
struct plus {
    auto operator()(const T& a, const T& b) const { return a + b; }
};

template <typename T>
struct less {
    auto operator()(const T& a, const T& b) const { return a < b; }
};

template <typename T>
struct greater {
    auto operator()(const T& a, const T& b) const { return a > b; }
};

template <typename T>
struct less_equal {
    auto operator()(const T& a, const T& b) const { return a <= b; }
};

template <typename T>
struct greater_equal {
    auto operator()(const T& a, const T& b) const { return a >= b; }
};

template <typename T>
struct identity {
    constexpr T&& operator()(T&& t) const noexcept { return std::forward<T>(t); }
    constexpr const T& operator()(const T& t) const noexcept { return t; }
};


// From https://en.cppreference.com/w/cpp/utility/variant/visit
// For constructing overloaded lambdas.

// helper type for the visitor #4
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

}
