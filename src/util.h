#pragma once

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <sstream>
#include <type_traits>
#include <numeric>
#include <iterator>
#include <exception>
#include <variant>

#include <llvm/IR/Function.h>
#include <llvm/Support/Format.h>

#include "binrel.h"
#include "assert-util.h"

extern char prog[];

template <typename... Args>
void log(const char *fmt, Args&&... args) {
    llvm::errs() << llvm::format(fmt, std::forward<Args>(args)...) << "\n";
}

template <typename... Args>
[[noreturn]] void error(const char *fmt, Args&&... args) {
    llvm::errs() << "error: ";
    log(fmt, std::forward<Args>(args)...);
    exit(1);
}

template <typename... Args>
void warning(const char *fmt, Args&&... args) {
    llvm::errs() << "warning: ";
    log(fmt, std::forward<Args>(args)...);
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

template <typename... Ts>
std::string to_string(Ts&&... xs) {
    std::stringstream ss;
    (ss << ... << xs);
    return ss.str();
}

template <typename T>
struct creator {
    template <typename... Args>
    T operator()(Args&&... args) const { return T {std::forward<Args>(args)...}; }
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



}






#define todo() std::cerr << __FILE__ << ":" << __LINE__ << ": todo\n"; std::abort()


namespace util {

template <class Key, class T, class Hash, class KeyEqual, class Allocator, class Combine>
std::unordered_map<Key, T, Hash, KeyEqual, Allocator>& update_assign(std::unordered_map<Key, T, Hash, KeyEqual, Allocator>& a, const std::unordered_map<Key, T, Hash, KeyEqual, Allocator>& b, Combine combine) {
    for (const auto& bp : b) {
        combine(a[bp.first], bp.second);
    }
    return a;
}

namespace detail {

template <typename Func>
struct defer_impl {
    Func func;
    ~defer_impl() { func(); }
};

}

template <typename Func>
detail::defer_impl<Func> defer(Func func) {
    return detail::defer_impl<Func> {.func = func};
}


template <typename Container>
class push_scope {
public:
    push_scope() {}
    push_scope(Container& container, const typename Container::value_type& x): container(&container) {
        container.push_back(x);
    }
    push_scope(push_scope&& other) {
        container = other.container; other.container = nullptr;
    }
    push_scope& operator=(push_scope&& other) {
        if (container) {
            pop();
        }
        container = other.container; other.container = nullptr;
    }
    
    ~push_scope() {
        if (container) {
            pop();
        }
    }
    
    bool good() const { return container != nullptr; }
    operator bool() const { return good(); }

private:
    Container *container = nullptr;
    
    void pop() {
        container->pop_back();
        container = nullptr;
    }
};

template <typename Container>
push_scope<Container> push(Container& container, const typename Container::value_type& x) {
    return push_scope<Container>(container, x);
}

template <typename T>
class op_scope {
public:
    using function_type = std::function<void (T&)>;
    op_scope(T& x, function_type inc, function_type dec): x(x), inc(inc), dec(dec) {
        inc(x);
    }
    
    ~op_scope() {
        dec(x);
    }
    
private:
    T& x;
    function_type inc;
    function_type dec;
};

template <typename T>
op_scope<T> inc_scope(T& x) {
    return op_scope<T> {
        x,
        [] (T& x) {
            ++x;
        },
        [] (T& x) {
            --x;
        },
    };
}

template <typename T>
class save_scope {
public:
    save_scope(T& orig): orig(orig), saved(orig) {}
    save_scope(const save_scope& other) = delete;
    ~save_scope() { orig = saved; }
private:
    T& orig;
    T saved;
};

template <typename T>
save_scope<T> save(T& x) {
    return save_scope<T>(x);
}

#define trace(...) \
do { \
fprintf(stderr, "%s:%d: ", __FUNCTION__, __LINE__); \
fprintf(stderr, __VA_ARGS__); \
fprintf(stderr, "\n"); \
} while (false)

}

