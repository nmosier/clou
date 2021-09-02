#pragma once

#include <tuple>
#include <vector>
#include <utility>

#include <z3++.h>

#include "noderef.h"
#include "uhb.h"
#include "aeg.h"
#include "default_map.h"

namespace fol {

/* It would be nice to be able to easily construct simple first-order logic expressions/formulas over the AEG
 * to verify that the implementation is correct. For example, we could verify that ~rf.co == fr.
 */

template <typename... Ts>
struct relation {
    using Tuple = std::tuple<Ts...>;
    struct Hash {
        auto operator()(const Tuple& tuple) const {
            return llvm::hash_value(tuple);
        }
    };
    using Map = std::unordered_map<Tuple, z3::expr, Hash>;
    
    Map map;
    
    using iterator = typename Map::iterator;
    using const_iterator = typename Map::const_iterator;
    using value_type = typename Map::value_type;
    
    template <typename... Args>
    const_iterator find(Args&&... args) const { return map.find(std::forward<Args>(args)...); }
    
    template <typename... Args>
    iterator find(Args&&... args) { return map.find(std::forward<Args>(args)...); }
    
    const_iterator begin() const { return map.begin(); }
    iterator begin() { return map.begin(); }
    
    const_iterator end() const { return map.end(); }
    iterator end() { return map.end(); }
    
    template <typename... Args>
    std::pair<iterator, bool> emplace(Args&&... args) { return map.emplace(std::forward<Args>(args)...); }
    
    template <typename... Args>
    auto insert(Args&&... args) { return map.insert(std::forward<Args>(args)...); }
    
    bool empty() const { return map.empty(); }
    std::size_t size() const { return map.size(); }
    
    template <typename UnaryOp>
    z3::expr update(const Tuple& tuple, const z3::expr& dfl, UnaryOp op) {
        auto it = find(tuple);
        if (it == end()) {
            it = emplace(tuple, dfl).first;
        }
        return it->second = op(it->second);
    }

    relation() = default;
    relation(const relation& other) = default;
    relation(relation&& other) = default;
};

template <typename... Ts>
relation<Ts...> make_relation(const std::tuple<Ts...>& example_tuple) {
    return relation<Ts...>();
}

using binary_node_relation = relation<NodeRef, NodeRef>;
using com_relation = binary_node_relation;
using address_relation = binary_node_relation;

binary_node_relation edge_rel(const AEG& aeg, UHBEdge::Kind kind);
relation<NodeRef> node_rel(const AEG& aeg, Inst::Kind kind);

template <typename Pred>
relation<NodeRef> node_rel(const AEG& aeg, Pred pred) {
    relation<NodeRef> rel;
    for (const auto p : aeg.node_range2()) {
        const z3::expr cond = pred(p);
        if (!cond.is_false()) {
            rel.emplace(std::make_tuple(p.ref), cond);
        }
    }
    return rel;
}

inline com_relation rf(const AEG& aeg) { return edge_rel(aeg, UHBEdge::RF); }
inline com_relation co(const AEG& aeg) { return edge_rel(aeg, UHBEdge::CO); }
inline com_relation fr(const AEG& aeg) { return edge_rel(aeg, UHBEdge::FR); }

template <typename T, typename U>
std::unordered_map<T, std::unordered_set<U>> to_map(const relation<T,U>& rel) {
    std::unordered_map<T, std::unordered_set<U>> map;
    for (const auto& p : rel) {
        const auto& tuple = p.first;
        map[std::get<0>(tuple)].insert(std::get<1>(tuple));
    }
    return map;
}

// relation union
template <typename... Ts>
relation<Ts...>& operator+=(relation<Ts...>& a, const relation<Ts...>& b) {
    for (const auto& pair : b) {
        const auto& tuple = pair.first;
        const auto it = a.find(tuple);
        if (it == a.end()) {
            a.insert(pair);
        } else {
            it->second = it->second || pair.second;
        }
    }
    return a;
}

template <typename... Ts>
relation<Ts...> operator+(const relation<Ts...>& a, const relation<Ts...>& b) {
    auto res = a;
    return res += b;
}

// relation difference
template <typename... Ts>
relation<Ts...> operator-=(relation<Ts...>& a, const relation<Ts...>& b) {
    for (const auto& pair : b) {
        const auto& tuple = pair.first;
        const auto it = a.find(tuple);
        if (it != a.end()) {
            it->second = it->second && !pair.second;
        }
    }
    return a;
}

template <typename... Ts>
relation<Ts...> operator-(const relation<Ts...>& a, const relation<Ts...>& b) {
    auto res = a;
    return res -= b;
}

template <typename... Ts>
relation<Ts...> operator&(const relation<Ts...>& a, const relation<Ts...>& b) {
    relation<Ts...> res;
    for (const auto& pair : a) {
        const auto& tuple = pair.first;
        const auto it = b.find(tuple);
        if (it != b.end()) {
            res.emplace(tuple, pair.second && it->second);
        }
    }
    return res;
}

template <typename... Ts>
relation<Ts...>& operator&=(relation<Ts...>& a, const relation<Ts...>& b) {
    return a = a & b;
}

template <size_t I, typename... Ts>
auto invert_tuple(const std::tuple<Ts...>& in) {
    constexpr size_t N = sizeof...(Ts);
    if constexpr (I == N) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(invert_tuple<I+1, Ts...>(in), std::make_tuple(std::get<I>(in)));
    }
}

template <typename... Ts>
auto invert_tuple(const std::tuple<Ts...>& in) {
    return invert_tuple<0, Ts...>(in);
}


template <std::size_t Begin, std::size_t End, typename... Ts>
auto get_tuple_slice(const std::tuple<Ts...>& in) {
    if constexpr (Begin == End) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(std::make_tuple(std::get<Begin>(in)), get_tuple_slice<Begin+1, End>(in));
    }
}


// relation inverse
template <typename... Ts>
auto inverse(const relation<Ts...>& in) {
    const auto out_tuple = invert_tuple(std::tuple<Ts...>());
    auto out = make_relation(out_tuple);
    std::transform(in.begin(), in.end(), std::inserter(out, out.end()), [] (const auto& in_pair) {
        return std::make_pair(invert_tuple(in_pair.first), in_pair.second);
    });
    return out;
}

template <typename R1, typename R2>
auto cartesian_product(const R1& a, const R2& b) {
    const auto example_out_tuple = std::tuple_cat(typename R1::Tuple {}, typename R2::Tuple {});
    auto out = make_relation(example_out_tuple);
    for (const auto& p1 : a) {
        for (const auto& p2 : b) {
            out.emplace(std::tuple_cat(p1.first, p2.first), p1.second && p2.second);
        }
    }
    return out;
}

template <typename R1, typename R2>
auto join(const R1& a, const R2& b) {
    using Tuple1 = typename R1::Tuple;
    using Tuple2 = typename R2::Tuple;
    constexpr std::size_t Size1 = std::tuple_size<Tuple1>();
    constexpr std::size_t Size2 = std::tuple_size<Tuple2>();
    using Type1 = decltype(std::get<Size1 - 1>(Tuple1 {}));
    using Type2 = decltype(std::get<0>(Tuple2 {}));
    static_assert(std::is_same<Type1, Type2>(), "incompatible tuple element types for join");
    const auto join_tuples = [] (const Tuple1& t1, const Tuple2& t2) {
        return std::tuple_cat(get_tuple_slice<0, Size1-1>(t1),
                              get_tuple_slice<1, Size2>  (t2));
    };
    
    const auto example_out_tuple = join_tuples(Tuple1 {}, Tuple2 {});
    auto out = make_relation(example_out_tuple);
    
    for (const auto& p1 : a) {
        const auto& t1 = p1.first;
        for (const auto& p2 : b) {
            const auto& t2 = p2.first;
            if (std::get<Size1-1>(t1) == std::get<0>(t2)) {
                const auto out_tuple = join_tuples(t1, t2);
                const auto it = out.find(out_tuple);
                if (it == out.end()) {
                    out.emplace(out_tuple, p1.second && p2.second);
                } else {
                    it->second = it->second || (p1.second && p2.second);
                }
            }
        }
    }
    
    return out;
}

// TODO: move this somewhere else, e.g. 'graphutils.h'
template <typename T>
class postorder {
private:
    using Set = std::unordered_set<T>;
    using Map = std::unordered_map<T, Set>;
public:
    using Rel = relation<T, T>;
    
    template <typename OutputIt>
    OutputIt operator()(const Map& map, OutputIt out) const {
        // construct map and node set
        Set srcs;
        Set dsts;
        for (const auto& p1 : map) {
            srcs.insert(p1.first);
            for (const T p2 : p1.second) {
                dsts.insert(p2);
            }
        }
        
        // find entry (in degree 0)
        T entry;
        for (const T src : srcs) {
            if (dsts.find(src) == dsts.end()) {
                entry = src;
                break;
            }
        }
        
        Set done;
        rec(map, done, out, entry);
        
        return out;
    }
private:
    template <typename OutputIt>
    bool rec(const Map& map, Set& done, OutputIt& out, T cur) const {
        if (done.find(cur) != done.end()) {
            return true;
        }
        bool acc = true;
        for (const T succ : map.at(cur)) {
            acc &= postorder_rec(map, done, out, succ);
        }
        
        if (acc) {
            done.insert(cur);
            *out++ = cur;
        }
        
        return acc;
    }
};


// Predicates
template <typename... Ts>
z3::expr some(const relation<Ts...>& rel, z3::context& ctx) {
    return util::any_of(rel.begin(), rel.end(), [] (const auto& p) -> z3::expr {
        return p.second;
    }, ctx.bool_val(false));
}

template <typename... Ts>
z3::expr all(const relation<Ts...>& rel, z3::context& ctx) {
    return util::all_of(rel.begin(), rel.end(), [] (const auto& p) -> z3::expr {
        return p.second;
    }, ctx.bool_val(true));
}

template <typename... Ts>
z3::expr one(const relation<Ts...>& rel, z3::context& ctx) {
    return util::one_of(rel.begin(), rel.end(), [] (const auto& p) -> z3::expr {
        return p.second;
    }, ctx.bool_val(true), ctx.bool_val(false));
}

template <typename... Ts>
z3::expr no(const relation<Ts...>& rel, z3::context& ctx) {
    return !some(rel, ctx);
}

template <typename... Ts>
z3::expr lone(const relation<Ts...>& rel, z3::context& ctx) {
    return util::lone_of(rel.begin(), rel.end(), [] (const auto& p) -> z3::expr {
        return p.second;
    }, ctx.bool_val(true), ctx.bool_val(false));
}

template <typename... Ts>
z3::expr subset(const relation<Ts...>& a, const relation<Ts...>& b, z3::context& ctx) {
    z3::expr acc = ctx.bool_val(true);
    for (const auto& p : a) {
        const auto it = b.find(p.first);
        z3::expr consequent {ctx};
        if (it == b.end()) {
            consequent = ctx.bool_val(false);
        } else {
            consequent = it->second;
        }
        acc = acc && z3::implies(p.second, consequent);
    }
    return acc;
}

// NOTE: We could write this as a bidirection subset, but it's more efficient to rewrite it as equality.
template <typename... Ts>
z3::expr equal(const relation<Ts...>& a, const relation<Ts...>& b, z3::context& ctx) {
    z3::expr acc = ctx.bool_val(true);
    const auto f = [&] (const auto& a, const auto& b) {
        for (const auto& p : a) {
            const auto& tuple = p.first;
            const auto it = b.find(tuple);
            if (it == b.end()) {
                acc = acc && !p.second;
            } else {
                acc = acc && p.second == it->second;
            }
        }
    };
    f(a, b);
    f(b, a);
    return acc;
}

template <typename... Ts>
z3::expr not_equal(const relation<Ts...>& a, const relation<Ts...>& b, z3::context& ctx) {
    return !equal(a, b, ctx);
}

template <unsigned N, typename... Ts>
auto element(const relation<Ts...>& a, z3::context& ctx) {
    using T = std::remove_reference_t<decltype(std::get<N>(std::tuple<Ts...> {}))>;
    relation<T> res;
    for (const auto& p : a) {
        const auto elt = std::get<N>(p.first);
        const auto it = res.find(elt);
        if (it != res.end()) {
            it->second = it->second || p.second;
        } else {
            res.emplace(elt, p.second);
        }
    }
    return res;
}

// transitive closure
template <typename T>
auto irreflexive_transitive_closure(const relation<T,T>& a) {
    // keep going until no new constraints added
    struct Hash {
        std::size_t operator()(const z3::expr& e) const { return e.id(); }
    };
    struct KeyEqual {
        bool operator()(const z3::expr& a, const z3::expr& b) const { return a.id() == b.id(); }
    };
    std::unordered_map<T, std::unordered_map<T, std::unordered_set<z3::expr, Hash, KeyEqual>>> map;
    
    // populate map
    for (const auto& p : a) {
        const auto& t = p.first;
        const T src = std::get<0>(t);
        const T dst = std::get<1>(t);
        map[src][dst].insert(p.second);
    }
    
    // iterative join
    bool changed;
    do {
        changed = false;
        for (const auto& p1 : map) {
            const T& n1 = p1.first;
            for (const auto& p2 : p1.second) {
                const T& n2 = p2.first;
                const auto& f1 = p2.second;
                for (const auto& p3 : map[n2]) {
                    const T& n3 = p3.first;
                    const auto& f2 = p3.second;
                    auto& fs = map[n1][n3];
                    const auto do_insert = [&] (const auto& f) {
                        for (const z3::expr& e : f) {
                            if (fs.insert(e).second) {
                                changed = true;
                            }
                        }
                    };
                    do_insert(f1);
                    do_insert(f2);
                }
            }
        }
    } while (changed);
    
    // convert map to relation
    relation<T,T> res;
    for (const auto& p1 : map) {
        for (const auto& p2 : p1.second) {
            const auto& fs = p2.second;
            assert(!fs.empty());
            const z3::expr f = std::reduce(std::next(fs.begin()), fs.end(), *fs.begin(),
                                           [] (const z3::expr& a, const z3::expr& b) {
                return a && b;
            });
            res.emplace(std::make_tuple(p1.first, p2.first), f);
        }
    }
    return res;
}

template <typename T>
relation<T,T> identity(const relation<T>& a) {
    relation<T,T> res;
    for (const auto& p : a) {
        res.emplace(std::tuple_cat(p.first, p.first), p.second);
    }
    return res;
}

template <typename T>
z3::expr cyclic(const relation<T,T>& a, z3::context& ctx) {
    const relation<T> srcs = element<0>(a, ctx);
    const relation<T,T> id = identity(srcs);
    const relation<T,T> closure = irreflexive_transitive_closure(a);
    return some(id & closure, ctx);
}

template <typename T>
z3::expr acyclic(const relation<T,T>& a, z3::context& ctx) {
    return !cyclic(a, ctx);
}

}
