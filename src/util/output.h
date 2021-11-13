#pragma once

#include <iostream>
#include <utility>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <vector>
#include <variant>
#include <sstream>

#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Format.h>
#include <llvm/IR/Function.h>

namespace output {

template <typename OutputStream, typename InputIt, typename Transform>
OutputStream& range(OutputStream& os, InputIt begin, InputIt end, const std::string& sep, Transform transform) {
   //  using ::operator<<;
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

inline std::ostream& operator<<(std::ostream& os, const llvm::Value& V) {
    std::string s;
    llvm::raw_string_ostream ss {s};
    ss << V;
    os << s;
    return os;
}

extern unsigned verbose;
#define logv(level, msg) \
do { \
if (verbose >= level) { \
llvm::errs() << msg; \
} \
} while (false)


extern char prog[];


#define log_(fmt, ...) do { \
std::fprintf(stderr, fmt __VA_OPT__(,) __VA_ARGS__); \
std::fprintf(stderr, "\n"); \
} while (false)


#define error(fmt, ...) do { \
log_(fmt __VA_OPT__(,) __VA_ARGS__); \
std::exit(1); \
} while (false)

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


namespace util {

template <typename... Ts>
std::string to_string(Ts&&... xs) {
    std::stringstream ss;
    (ss << ... << xs);
    return ss.str();
}


}


#define trace(...) \
do { \
fprintf(stderr, "%s:%d: ", __FUNCTION__, __LINE__); \
fprintf(stderr, __VA_ARGS__); \
fprintf(stderr, "\n"); \
} while (false)


#define todo() std::cerr << __FILE__ << ":" << __LINE__ << ": todo\n"; std::abort()
