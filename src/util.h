#pragma once

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <sstream>
#include <type_traits>
#include <numeric>
#include <iterator>
#include <exception>

#include <llvm/IR/Function.h>
#include <llvm/Support/Format.h>

#include "binrel.h"
#include "assert-util.h"

extern char prog[];

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
#ifdef __APPLE__
    return format(fmtcheck(fmt.c_str(), "%s.dot"), F.getName().str().c_str());
#else
    return format(fmt.c_str(), F.getName().str().c_str());
#endif
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

template <typename T, typename InputIt, typename UnaryPredicate>
T any_of(InputIt begin, InputIt end, UnaryPredicate p, T false_val) {
    return std::transform_reduce(begin, end, false_val,
                                 [] (const T& a, const T& b) -> T {
        return a || b;
    }, p);
}

template <typename T, typename InputIt, typename UnaryPredicate>
std::vector<T> count_of(InputIt begin, InputIt end, UnaryPredicate p, T true_value, T false_value) {
    std::vector<T> prev_zero;
    std::vector<T> prev_one;
    for (auto it = begin; it != end; ++it) {
        const T t = p(*it);
        prev_zero.push_back(!t);
        prev_one.push_back(t);
    }
    
    if (prev_zero.empty()) {
        return {true_value, false_value};
    }
    
    // pad to the nearest power of 2
    while ((prev_zero.size() & (prev_zero.size() - 1)) != 0) {
        prev_zero.push_back(true_value);
        prev_one.push_back(false_value);
    }
    
    while (prev_zero.size() > 1) {
        std::vector<T> cur_zero, cur_one;
        for (std::size_t n = 0; n < prev_zero.size() / 2; ++n) {
            const auto l = n * 2;
            const auto r = l + 1;
            cur_zero.push_back(prev_zero.at(l) && prev_zero.at(r));
            cur_one.push_back((prev_one.at(l) && prev_zero.at(r)) ||
                              (prev_zero.at(l) && prev_one.at(r)));
        }
        prev_zero = std::move(cur_zero);
        prev_one = std::move(cur_one);
    }

    assert_eq(prev_one.size(), (std::size_t) 1);
    return {prev_zero.front(), prev_one.front()};
}

template <typename T, typename InputIt, typename UnaryPredicate>
T lone_of(InputIt begin, InputIt end, UnaryPredicate p, T true_val, T false_val) {
    const auto res = zero_one_of(begin, end, p, true_val, false_val);
    return res.at(0) || res.at(1);
}

template <typename T, typename InputIt, typename UnaryPredicate>
T one_of(InputIt begin, InputIt end, UnaryPredicate p, T true_val, T false_val) {
    return count_of(begin, end, p, true_val, false_val).at(1);
}

template <typename T, typename InputIt, typename UnaryPredicate>
T none_of(InputIt begin, InputIt end, UnaryPredicate p, T true_val, T false_val) {
    return !util::any_of(begin, end, p, false_val);
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

template <typename Container, typename... Args>
std::optional<typename Container::const_iterator> contains(const Container& container, Args&&... args) {
    const auto it = container.find(std::forward<Args>(args)...);
    return it == container.end() ? std::nullopt : std::make_optional(it);
}

template <typename Container, typename... Args>
std::optional<typename Container::iterator> contains(Container& container, Args&&... args) {
    const auto it = container.find(std::forward<Args>(args)...);
    return it == container.end() ? std::nullopt : it;
}

}

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

namespace util {

template <std::size_t I, typename Func, typename... Ts>
void for_each_in_tuple(const std::tuple<Ts...>& tuple, Func func) {
    if constexpr (I < sizeof...(Ts)) {
        func(std::get<I>(tuple));
        for_each_in_tuple<I+1>(tuple, func);
    }
}

template <typename Func, typename... Ts>
void for_each_in_tuple(const std::tuple<Ts...>& tuple, Func func) {
    return for_each_in_tuple<0>(tuple, func);
}

template <typename Container1, typename Container2, typename OutputIt>
OutputIt set_intersection(const Container1& small, const Container2& large, OutputIt out) {
    if (small.size() > large.size()) {
        return set_intersection(large, small, out);
    }

    for (const auto& x : small) {
        if (large.find(x) != large.end()) {
            *out++ = x;
        }
    }
    
    return out;
}

}

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


namespace util {
template <typename T>
struct unordered_pair {
    T first;
    T second;
    
    unordered_pair() {}
    unordered_pair(const T& first, const T& second): first(first), second(second) {}
    
    bool operator==(const unordered_pair& other) const {
        return (first == other.first && second == other.second) || (first == other.second && second == other.first);
    }
    
    bool operator!=(const unordered_pair& other) const {
        return !(*this == other);
    }
    
    std::size_t hash() const {
        if (!(second < first)) {
            return llvm::hash_value(std::make_pair(first, second));
        } else {
            return llvm::hash_value(std::make_pair(second, first));
        }
    }
};

}

namespace util {

struct resume: std::exception {
    virtual const char *what() const noexcept {
        return msg.c_str();
    }
    
    std::string msg;
    
    resume(const std::string& msg): msg(msg) {}
};

inline std::system_error syserr(const std::string& what = "") {
    return std::system_error(std::error_code(errno, std::generic_category()), what);
}

struct null_output_iterator {
    const null_output_iterator& operator++() const { return *this; }
    const null_output_iterator& operator++(int) const { return *this; }
    const null_output_iterator& operator*() const { return *this; }
    
    template <typename T>
    const T& operator=(const T& val) const { return val; }
};

}
