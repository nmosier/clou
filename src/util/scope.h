#pragma once

#include "util/fwd/scope.h"

namespace util {


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

}
