#pragma once

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
T none_of(InputIt begin, InputIt end, UnaryPredicate p, T true_val, T false_val) {
    return !util::any_of(begin, end, p, false_val);
}


}
