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


}
