#pragma once

#include <tuple>

/** std::tuple utilities. */
namespace tuples {

template <std::size_t I, typename... Ts>
auto invert(const std::tuple<Ts...>& in) {
    constexpr std::size_t N = sizeof...(Ts);
    if constexpr (I == N) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(invert<I+1, Ts...>(in), std::make_tuple(std::get<I>(in)));
    }
}

/** Invert a tuple.
 * \return \f$ (a_n, \ldots, a_1) \text{ where } a = (a_1, \ldots, a_n) \f$ */
template <typename Bool, typename... Ts>
auto invert(const std::tuple<Bool, Ts...>& a) {
    return invert<0, Bool, Ts...>(a);
}

/** Get a tuple slice from index \p Begin to index \p End (half-open interval).
 * \return \f$  (a_i, \ldots, a_{j-1}) \text{ where } (a_1, \ldots, a_n) = a, i = \texttt{Begin}, j = \texttt{End} \f$ */
template <std::size_t Begin, std::size_t End, typename Bool, typename... Ts>
auto slice(const std::tuple<Ts...>& in) {
    if constexpr (Begin == End) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(std::make_tuple(std::get<Begin>(in)), slice<Begin+1, End, Bool>(in));
    }
}

/** Make a tuple of repeating values.
 * \return \f$ (a_1, a_2, \ldots, a_N) \text{ where } a_i = \texttt{Element} \f$ */
template <std::size_t N, typename T>
auto make_repeated(const T& element) {
    if constexpr (N == 0) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(std::make_tuple(element), make_repeated<N-1>(element));
    }
}

namespace detail {

template <typename std::size_t I, typename Func, typename... Ts>
auto transform_impl(const std::tuple<Ts...>& tuple, Func func) {
    if constexpr (I == sizeof...(Ts)) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(std::make_tuple(func(std::get<I>(tuple))),
                              transform_impl<I+1>(tuple, func));
    }
}

}

template <typename Func, typename... Ts>
auto transform(const std::tuple<Ts...>& tuple, Func func) {
    return detail::transform_impl<0>(tuple, func);
}


namespace detail {

template <std::size_t I, typename Func, typename... Ts>
void for_each_impl(const std::tuple<Ts...>& tuple, Func func) {
    if constexpr (I < sizeof...(Ts)) {
        func(std::get<I>(tuple));
        for_each_impl<I+1>(tuple, func);
    }
}

}

template <typename Func, typename... Ts>
void for_each(const std::tuple<Ts...>& tuple, Func func) {
    return detail::for_each_impl<0>(tuple, func);
}

}
