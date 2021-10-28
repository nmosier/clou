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


}
