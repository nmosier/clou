#pragma once

#include <functional>

namespace util {

#if 1
template <typename Container, typename... Args>
std::optional<typename Container::const_iterator> contains(const Container& container, Args&&... args) {
    const auto it = container.find(std::forward<Args>(args)...);
    return it == container.end() ? std::nullopt : std::make_optional(it);
}

template <typename Container, typename... Args>
std::optional<typename Container::iterator> contains(Container& container, Args&&... args) {
    const auto it = container.find(std::forward<Args>(args)...);
    return it == container.end() ? std::nullopt : std::make_optional(it);
}
#endif


struct null_output_iterator {
    const null_output_iterator& operator++() const { return *this; }
    const null_output_iterator& operator++(int) const { return *this; }
    const null_output_iterator& operator*() const { return *this; }
    
    template <typename T>
    const T& operator=(const T& val) const { return val; }
};




template <typename T, typename Derived>
struct BaseOutputIterator {
    Derived& operator++() { return cast(); }
    Derived& operator++(int) { return cast(); }
    Derived& operator*() { return cast(); }
    Derived& operator=(const T& x) {
        cast().put(x);
        return cast();
    }
    
private:
    Derived& cast() {
        return static_cast<Derived&>(*this);
    }
};


template <typename T>
struct FunctionOutputIterator: BaseOutputIterator<T, FunctionOutputIterator<T>> {
    using function_type = std::function<void (const T&)>;
    function_type func;
    FunctionOutputIterator(function_type func): func(func) {}
    void put(const T& x) {
        func(x);
    }
};

template <typename T, typename U, typename OutputIt>
struct TransformOutputIterator: FunctionOutputIterator<T> {
    using function_type = std::function<U (const T&)>;
    OutputIt out;
    
    TransformOutputIterator(OutputIt out, function_type op): FunctionOutputIterator<T>([&, op] (const T& x) {
        *this->out++ = op(x);
    }), out(out) {}
};

template <typename T, typename U, typename OutputIt>
TransformOutputIterator<T, U, OutputIt> make_transform_output_iterator(OutputIt out, std::function<U (const T&)> op) {
    return TransformOutputIterator<T, U, OutputIt>(out, op);
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



template <typename InputIt, typename Func>
void for_each_unordered_pair(InputIt first, InputIt last, Func func) {
    for (auto it1 = first; it1 != last; ++it1) {
        for (auto it2 = std::next(it1); it2 != last; ++it2) {
            func(*it1, *it2);
        }
    }
}

template <class Container, class Func>
void for_each_unordered_pair(const Container& container, Func func) {
    for_each_unordered_pair(container.begin(), container.end(), func);
}


}
