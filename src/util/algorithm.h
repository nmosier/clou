#pragma once

#include <numeric>

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


template <typename T>
class complete_subgraphs {
public:
    using Set = std::unordered_set<T>;
    using Map = std::unordered_map<T, Set>;
    
    complete_subgraphs(const Map& map): map(map) {}
    
    template <typename OutputIt>
    OutputIt operator()(OutputIt out) {
        for (auto root_it = map.begin(); root_it != map.end(); ++root_it) {
            auto& root_pair = *root_it;
            const T& root = root_pair.first;
            Set& root_succs = root_pair.second;
            
            while (!root_succs.empty()) {
                std::vector<MapIter> group = {root_it};
                for (auto root_succ_it = root_succs.begin(); root_succ_it != root_succs.end(); ++root_succ_it) {
                    const T& x = *root_succ_it;
                    bool add_x = true;
                    for (const MapIter& y : group) {
                        if (y->second.find(x) == y->second.end()) {
                            add_x = false;
                            break;
                        }
                    }
                    if (add_x) {
                        group.push_back(map.find(x));
                    }
                }
                
                // output subgraph
                Set subgraph;
                std::transform(group.begin(), group.end(), std::inserter(subgraph, subgraph.end()), [] (const MapIter& x) -> T {
                    return x->first;
                });
                *out++ = subgraph;
                
                // erase captured edges
                for (const T& x : subgraph) {
                    Set& x_succs = map.at(x);
                    for (const T& y : subgraph) {
                        x_succs.erase(y);
                    }
                }
            }
        }
        
        return out;
    }
    
private:
    using MapIter = typename Map::iterator;
    using MapIters = std::vector<MapIter>;
    using SetIter = typename Set::iterator;
    
    Map map;
    
};


template <typename Container, typename OutputIt>
OutputIt copy(const Container& container, OutputIt out) {
    return std::copy(container.begin(), container.end(), out);
}

template <typename Container, typename OutputIt, typename UnaryOp>
OutputIt transform(const Container& container, OutputIt out, UnaryOp op) {
    return std::transform(container.begin(), container.end(), out, op);
}

template <typename A, typename B>
bool subset(const A& a, const B& b) {
    for (const auto& x : a) {
        if (b.find(x) == b.end()) {
            return false;
        }
    }
    return true;
}


/** Check whether \p a is a proper prefix of \p b. */
template <typename T>
bool prefix(const std::vector<T>& a, const std::vector<T>& b) {
    if (a.size() >= b.size()) {
        return false;
    }
    return std::equal(a.begin(), a.end(), b.begin());
}

/** Check whether \p a is a prefix of or equal to \p b. */
template <typename T>
bool prefixeq(const std::vector<T>& a, const std::vector<T>& b) {
    if (a.size() > b.size()) {
        return false;
    }
    return std::equal(a.begin(), a.end(), b.begin());
}

template <typename Container>
std::optional<typename Container::value_type> get_singleton(const Container& container) {
    if (container.size() == 1) {
        return *container.begin();
    } else {
        return std::nullopt;
    }
}

template <typename Container, typename T, typename BinaryOp, typename UnaryOp>
T transform_reduce(const Container& container, T init, BinaryOp b, UnaryOp u) {
    return std::transform_reduce(container.begin(), container.end(), init, b, u);
}

template <typename T, typename Container, typename UnaryOp>
T transform_min(const Container& container, UnaryOp u) {
    return util::transform_reduce(container, std::numeric_limits<T>::max(), [] (T a, T b) -> T {
        return std::min<T>(a, b);
    }, u);
}


}
