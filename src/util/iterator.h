#pragma once

namespace util {

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




}
