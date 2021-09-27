#pragma once

#include <tuple>
#include <vector>
#include <utility>

#include <z3++.h>

#include "noderef.h"
#include "uhb.h"
#include "aeg.h"
#include "default_map.h"
#include "util.h"

namespace fol {

/* It would be nice to be able to easily construct simple first-order logic expressions/formulas over the AEG
 * to verify that the implementation is correct. For example, we could verify that ~rf.co == fr.
 */

template <typename Bool>
struct Logic {};

template <>
struct Logic<bool> {
    static constexpr bool T = true;
    static constexpr bool F = false;
    bool is_true(bool b) const { return b; }
    bool is_false(bool b) const { return !b; }
    bool implies(bool pred, bool cons) const { return !pred || cons; }
    
    using Hash = std::hash<bool>;
    using Equal = std::equal_to<bool>;
};

template <>
struct Logic<z3::expr> {
    z3::expr T, F;
    bool is_true(const z3::expr& b) const { return b.simplify().is_true(); }
    bool is_false(const z3::expr& b) const { return b.simplify().is_false(); }
    bool implies(const z3::expr& pred, const z3::expr& cons) const { return z3::implies(pred, cons); }
    Logic(z3::context& ctx): T(ctx.bool_val(true)), F(ctx.bool_val(false)) {}
    
    struct Hash {
        std::size_t operator()(const z3::expr& e) const { return e.id(); }
    };
    struct Equal {
        bool operator()(const z3::expr& a, const z3::expr& b) const { return a.id() == b.id(); }
    };
};

template <typename Bool, typename... Ts>
struct relation {
    using Tuple = std::tuple<Ts...>;
    struct Hash {
        auto operator()(const Tuple& tuple) const {
            return llvm::hash_value(tuple);
        }
    };
    using Map = std::unordered_map<Tuple, Bool, Hash>;
    using logic_type = Logic<Bool>;
    using bool_type = Bool;
    
    logic_type logic;
    Map map;
    
    using const_iterator = typename Map::const_iterator;
    using value_type = typename Map::value_type;
    
    template <typename... Args>
    const_iterator find(Args&&... args) const { return map.find(std::forward<Args>(args)...); }
    
    const_iterator begin() const { return map.begin(); }
    const_iterator end() const { return map.end(); }
    
    void emplace(const Tuple& tuple, const Bool& cond) {
        if (!logic.is_false(cond)) {
            map.emplace(tuple, cond);
        }
    }
    
    void insert(const std::pair<Tuple, Bool>& pair) {
        if (!logic.is_false(pair.second)) {
            map.insert(pair);
        }
    }
    
    void insert_or_assign(const Tuple& t, const Bool& b) {
        if (logic.is_false(b)) {
            map.erase(t);
        } else {
            map.insert_or_assign(t, b);
        }
    }
    
    const Bool& at(const Tuple& t) const { return map.at(t); }
    
    bool empty() const { return map.empty(); }
    std::size_t size() const { return map.size(); }
    
    void update(const Tuple& tuple, const Bool& b) {
        const auto it = map.find(tuple);
        if (logic.is_false(b)) {
            map.erase(it);
        } else {
            it->second = b;
        }
    }
    
    // get by tuple, defaults to false if tuple not in map
    Bool get(const Tuple& t) const {
        const auto it = find(t);
        return it == end() ? logic.F : it->second;
    }
    
    relation(const logic_type& logic = logic_type()): logic(logic) {}
    relation(const relation& other) = default;
    relation(relation&& other) = default;
};


template <typename Bool, typename Eval = util::identity<Bool>>
struct Context {
    using bool_type = Bool;
    using logic_type = Logic<Bool>;
    template <typename... Ts>
    using relation_type = relation<Bool, Ts...>;
    logic_type logic;
    Eval eval;
    AEG& aeg;
    
    template <typename... Ts>
    relation_type<Ts...> make_relation(const std::tuple<Ts...>& example_tuple) const {
        return relation_type<Ts...>(logic);
    }
    
    relation_type<NodeRef, NodeRef> edge_rel(UHBEdge::Kind kind) const;
    
    relation_type<NodeRef> node_rel(Inst::Kind kind, ExecMode mode);
    relation_type<NodeRef> node_rel(ExecMode mode);
    template <typename Pred>
    relation_type<NodeRef> node_rel_if(Pred pred) {
        relation_type<NodeRef> rel {logic};
        for (const NodeRef ref : aeg.node_range()) {
            const Bool cond = eval(pred(ref, aeg.lookup(ref)));
            if (!logic.is_false(cond)) {
                rel.emplace(std::make_tuple(ref), cond);
            }
        }
        return rel;
    }
    
    template <typename Pred>
    relation_type<NodeRef, NodeRef> binary_rel_if(Pred pred) const {
        relation_type<NodeRef, NodeRef> rel;
        for (const NodeRef ref1 : aeg.node_range()) {
            for (const NodeRef ref2 : aeg.node_range()) {
                const Bool cond = eval(pred(ref1, ref2));
                rel.emplace(std::make_tuple(ref1, ref2), cond);
            }
        }
        return rel;
    }
    
    relation_type<NodeRef, NodeRef> co() const;
    relation_type<NodeRef, NodeRef> fr() const;
    
    relation_type<NodeRef, NodeRef> same_addr() const;
    relation_type<NodeRef, NodeRef> same_xstate() const;
    
    template <typename Func, typename... Ts>
    relation_type<Ts...> filter(const relation_type<Ts...>& a, Func func) const {
        relation_type<Ts...> res {logic};
        for (const auto& pair : a) {
            res.emplace(pair.first, eval(func(pair.first)) && pair.second);
        }
        return res;
    }
    
    Context(const logic_type& logic, Eval eval, AEG& aeg): logic(logic), eval(eval), aeg(aeg) {}
};

template <typename Bool, typename... Ts>
relation<Bool, Ts...> make_relation(const std::tuple<Ts...>& example_tuple, const Logic<Bool>& logic) {
    return relation<Bool, Ts...>(logic);
}

struct SymEval {
    z3::context *ctx;
    z3::expr operator()(bool b) const { return ctx->bool_val(b); }
    const z3::expr& operator()(const z3::expr& e) const { return e; }
    SymEval(z3::context& ctx): ctx(&ctx) {}
};

struct ConEval {
    z3::eval eval;
    bool operator()(bool b) const { return b; }
    bool operator()(const z3::expr& e) const { return eval(e); }
    ConEval(const z3::eval& eval): eval(eval) {}
};

#if 0
template <typename T, typename U>
std::unordered_map<T, std::unordered_set<U>> to_map(const relation<T,U>& rel) {
    std::unordered_map<T, std::unordered_set<U>> map;
    for (const auto& p : rel) {
        const auto& tuple = p.first;
        map[std::get<0>(tuple)].insert(std::get<1>(tuple));
    }
    return map;
}
#endif

// relation union
template <typename Bool, typename... Ts>
relation<Bool, Ts...>& operator+=(relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    for (const auto& pair : b) {
        const auto& tuple = pair.first;
        a.insert_or_assign(tuple, a.get(tuple) || pair.second);
    }
    return a;
}

template <typename Bool, typename... Ts>
relation<Bool, Ts...> operator+(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    auto res = a;
    return res += b;
}

// relation difference
template <typename Bool, typename... Ts>
relation<Bool, Ts...> operator-=(relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    for (const auto& pair : b) {
        const auto& tuple = pair.first;
        a.insert_or_assign(tuple, a.get(tuple) && !pair.second);
    }
    return a;
}

template <typename Bool, typename... Ts>
relation<Bool, Ts...> operator-(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    auto res = a;
    return res -= b;
}

template <typename Bool, typename... Ts>
relation<Bool, Ts...> operator&(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    relation<Bool, Ts...> res {a.logic};
    for (const auto& pair : a) {
        const auto& tuple = pair.first;
        const auto it = b.find(tuple);
        if (it != b.end()) {
            res.emplace(tuple, pair.second && it->second);
        }
    }
    return res;
}

template <typename Bool, typename... Ts>
relation<Bool, Ts...>& operator&=(relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    return a = a & b;
}

template <std::size_t I, typename... Ts>
auto invert_tuple(const std::tuple<Ts...>& in) {
    constexpr std::size_t N = sizeof...(Ts);
    if constexpr (I == N) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(invert_tuple<I+1, Ts...>(in), std::make_tuple(std::get<I>(in)));
    }
}

template <typename Bool, typename... Ts>
auto invert_tuple(const std::tuple<Bool, Ts...>& in) {
    return invert_tuple<0, Bool, Ts...>(in);
}


template <std::size_t Begin, std::size_t End, typename Bool, typename... Ts>
auto get_tuple_slice(const std::tuple<Ts...>& in) {
    if constexpr (Begin == End) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(std::make_tuple(std::get<Begin>(in)), get_tuple_slice<Begin+1, End, Bool>(in));
    }
}


// relation inverse
template <typename Bool, typename... Ts>
auto inverse(const relation<Bool, Ts...>& in) {
    const auto out_tuple = invert_tuple(std::tuple<Ts...>());
    auto out = make_relation(out_tuple, in.logic);
    std::transform(in.begin(), in.end(), std::inserter(out.map, out.map.end()), [] (const auto& in_pair) {
        return std::make_pair(invert_tuple(in_pair.first), in_pair.second);
    });
    return out;
}

template <typename Bool, typename... Ts>
auto operator~(const relation<Bool, Ts...>& a) {
    return inverse(a);
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
    using Bool = typename R1::bool_type;
    constexpr std::size_t Size1 = std::tuple_size<Tuple1>();
    constexpr std::size_t Size2 = std::tuple_size<Tuple2>();
    using Type1 = decltype(std::get<Size1 - 1>(Tuple1 {}));
    using Type2 = decltype(std::get<0>(Tuple2 {}));
    static_assert(std::is_same<Type1, Type2>(), "incompatible tuple element types for join");
    const auto join_tuples = [] (const Tuple1& t1, const Tuple2& t2) {
        return std::tuple_cat(get_tuple_slice<0, Size1-1, Bool>(t1),
                              get_tuple_slice<1, Size2, Bool>  (t2));
    };
    
    const auto example_out_tuple = join_tuples(Tuple1 {}, Tuple2 {});
    auto out = make_relation<Bool>(example_out_tuple, a.logic);
    
    for (const auto& p1 : a) {
        const auto& t1 = p1.first;
        for (const auto& p2 : b) {
            const auto& t2 = p2.first;
            if (std::get<Size1-1>(t1) == std::get<0>(t2)) {
                const auto out_tuple = join_tuples(t1, t2);
                out.insert_or_assign(out_tuple, out.get(out_tuple) || (p1.second && p2.second));
            }
        }
    }
    
    return out;
}

template <std::size_t idx, typename Bool, typename T, typename... Ts>
relation<Bool, Ts...> restrict_element(const relation<Bool, Ts...>& a, const relation<Bool, T>& b) {
    static_assert(std::is_same<T, typename std::tuple_element<idx, std::tuple<Ts...>>::type>());
    const auto& L = a.logic;
    relation<Bool, Ts...> res {L};
    for (const auto& pair : a) {
        res.emplace(pair.first, pair.second && b.get(std::make_tuple(std::get<idx>(pair.first))));
    }
    return res;
}

#if 0
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
#endif


// Predicates
template <typename Bool, typename... Ts>
Bool some(const relation<Bool, Ts...>& rel) {
    return util::any_of(rel.begin(), rel.end(), [] (const auto& p) -> Bool {
        return p.second;
    }, rel.logic.F);
}

template <typename Bool, typename... Ts>
Bool all(const relation<Bool, Ts...>& rel) {
    return util::all_of(rel.begin(), rel.end(), [] (const auto& p) -> Bool {
        return p.second;
    }, rel.logic.T);
}

template <typename Bool, typename... Ts>
Bool one(const relation<Bool, Ts...>& rel) {
    if constexpr (std::is_same<Bool, z3::expr>()) {
        z3::expr_vector vec {rel.logic.T.ctx()};
        for (const auto& pair : rel) {
            vec.push_back(pair.second);
        }
        return z3::exactly(vec, 1);
    } else {
        return util::one_of(rel.begin(), rel.end(), [] (const auto& p) -> Bool { return p.second; }, rel.logic.T, rel.logic.F);
    }
}

template <typename Bool, typename... Ts>
Bool no(const relation<Bool, Ts...>& rel) {
    return !some(rel);
}

template <typename Bool, typename... Ts>
Bool lone(const relation<Bool, Ts...>& rel) {
    if constexpr (std::is_same<Bool, z3::expr>()) {
        z3::expr_vector vec {rel.logic.T.ctx()};
        for (const auto& pair : rel) {
            vec.push_back(pair.second);
        }
        return z3::atmost(vec, 1);
    } else {
        return util::lone_of(rel.begin(), rel.end(), [] (const auto& p) -> Bool { return p.second; }, rel.logic.T, rel.logic.F);
    }
}

template <typename Bool, typename... Ts>
Bool subset(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    const auto& L = a.logic;
    Bool acc = L.T;
    for (const auto& p : a) {
        const auto it = b.find(p.first);
        const Bool cons = it == b.end() ? L.F : it->second;
        acc = acc && L.implies(p.second, cons);
    }
    return acc;
}

// NOTE: We could write this as a bidirection subset, but it's more efficient to rewrite it as equality.
template <typename Bool, typename... Ts>
Bool equal(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    const auto& L = a.logic;
    Bool acc = L.T;
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

template <typename Bool, typename... Ts>
Bool not_equal(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    return !equal(a, b);
}

template <unsigned N, typename Bool, typename... Ts>
auto element(const relation<Bool, Ts...>& a) {
    using T = std::remove_reference_t<decltype(std::get<N>(std::tuple<Ts...> {}))>;
    relation<Bool, T> res;
    for (const auto& p : a) {
        const auto elt = std::get<N>(p.first);
        res.insert_or_assign(elt, res.get(elt) || p.second);
    }
    return res;
}

// transitive closure
template <typename Bool, typename T>
relation<Bool, T, T> irreflexive_transitive_closure(const relation<Bool, T, T>& a) {
    // keep going until no new constraints added
    std::unordered_map<T, std::unordered_map<T, std::unordered_set<Bool, typename Logic<Bool>::Hash, typename Logic<Bool>::Equal>>> map;
    
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
                        for (const auto& e : f) {
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
    relation<Bool, T, T> res {a.logic};
    for (const auto& p1 : map) {
        for (const auto& p2 : p1.second) {
            const auto& fs = p2.second;
            assert(!fs.empty());
            const Bool f = util::all_of(std::next(fs.begin()), fs.end(), util::logical_and<Bool>(), a.logic.T);
            res.emplace(std::make_tuple(p1.first, p2.first), f);
        }
    }
    return res;
}

template <std::size_t N, typename T>
auto make_repeated_tuple(const T& element) {
    if constexpr (N == 0) {
        return std::make_tuple();
    } else {
        return std::tuple_cat(std::make_tuple(element), make_repeated_tuple<N-1>(element));
    }
}

template <std::size_t N = 2, typename Bool, typename T>
auto identity(const relation<Bool, T>& a) {
    auto res = make_relation(make_repeated_tuple<N, T>(T()), a.logic);
    for (const auto& pair : a) {
        res.emplace(make_repeated_tuple<N>(std::get<0>(pair.first)), pair.second);
    }
    return res;
}

template <typename Bool, typename... Ts>
auto extend_last(const relation<Bool, Ts...>& a) {
    constexpr std::size_t last = sizeof...(Ts) - 1;
    using Last = typename std::tuple_element<last, std::tuple<Ts...>>::type;
    relation<bool, Ts..., Last> res {a.logic};
    for (const auto& pair : a) {
        res.emplace(std::tuple_cat(pair.first, std::make_tuple(std::get<last>(pair.first))), pair.second);
    }
    return res;
}

template <typename Rel1, typename Rel2>
auto join2(const Rel1& a_, const Rel2& b) {
    const auto a = extend_last(a_);
    return join(a, b);
}

template <typename Bool, typename T>
Bool cyclic(const relation<T,T>& a) {
    const relation<Bool, T> srcs = element<0>(a);
    const relation<Bool, T, T> id = identity(srcs);
    const relation<Bool, T, T> closure = irreflexive_transitive_closure(a);
    return some(id & closure);
}

template <typename Bool, typename T>
Bool acyclic(const relation<Bool, T, T>& a) {
    return !cyclic(a);
}

template <typename Bool, typename... Ts>
using for_func = std::function<Bool (const relation<Bool, Ts...>&)>;

template <typename Bool, typename... Ts>
Bool for_all(const relation<Bool, Ts...>& rel, const for_func<Bool, Ts...>& f) {
    const auto& L = rel.logic;
    return util::all_of(rel.begin(), rel.end(), [&] (const auto& pair) {
        relation<Bool, Ts...> item {L};
        item.map.insert(pair);
        return L.implies(pair.second, f(item));
    }, L.T);
}

template <typename Bool, typename... Ts>
Bool for_some(const relation<Bool, Ts...>& rel, const for_func<Bool, Ts...>& f) {
    return util::any_of(rel.begin(), rel.end(), [&] (const auto& pair) {
        return pair.second && f(pair.first);
    }, rel.logic.F);
}

template <typename Bool, typename... Ts>
Bool for_one(const relation<Bool, Ts...>& rel, const for_func<Bool, Ts...>& f) {
    const auto& L = rel.logic;
    return util::one_of(rel.begin(), rel.end(), [&] (const auto& pair) {
        return pair.second && f(pair.first);
    }, L.T, L.F);
}

template <typename Bool, typename... Ts>
Bool for_lone(const relation<Bool, Ts...>& rel, const for_func<Bool, Ts...>& f) {
    const auto& L = rel.logic;
    return util::lone_of(rel.begin(), rel.end(), [&] (const auto& pair) {
        return pair.second && f(pair.first);
    }, L.T, L.F);
}

template <typename Bool, typename... Ts>
Bool for_none(const relation<Bool, Ts...>& rel, const for_func<Bool, Ts...>& f) {
    const auto& L = rel.logic;
    return util::none_of(rel.begin(), rel.end(), [&] (const auto& pair) {
        return pair.second && f(pair.first);
    }, L.T, L.F);
}



template <typename Bool, typename Eval>
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::co() const {
    NodeRefVec writes;
    aeg.get_writes(std::back_inserter(writes));
    
    relation<Bool, NodeRef, NodeRef> rel {logic};
    for (auto it1 = writes.begin(); it1 != writes.end(); ++it1) {
        for (auto it2 = writes.begin(); it2 != writes.end(); ++it2) {
            const auto f = aeg.co_exists(*it1, *it2);
            if (!f.is_false()) {
                const auto tuple = std::make_tuple(*it1, *it2);
                const AEG::Node& src = aeg.lookup(*it1);
                const AEG::Node& dst = aeg.lookup(*it2);
                rel.emplace(tuple, eval(src.arch && dst.arch && src.same_addr(dst)));
            }
        }
    }
    
    return rel;
}

#if 0
template <typename Bool, typename Eval>
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::rf() const {
    using Node = UHBNode;
    NodeRefVec order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    relation<Bool, NodeRef, NodeRef> rel {logic};
    using ::operator<<;
    std::cerr << "ORDER: " << order << "\n";
    for (auto it1 = order.begin(); it1 != order.end(); ++it1) {
        const Node& src = aeg.lookup(*it1);
        if (!src.is_write()) { continue; }
        for (auto it2 = std::next(it1); it2 != order.end(); ++it2) {
            const Node& dst = aeg.lookup(*it2);
            if (!dst.is_read()) { continue; }
            if (*it1 == 0) {
                std::cerr << "rf_exists(0, " << *it2 << ") = " << aeg.rf_exists(*it1, *it2) << "\n";
            }
            const z3::expr exists = aeg.rf_exists(*it1, *it2);
            rel.emplace(std::make_tuple(*it1, *it2), eval(exists));
        }
    }
    return rel;
}
#endif

template <typename Bool, typename Eval>
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::fr() const {
    using Node = UHBNode;
    NodeRefVec order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    relation_type<NodeRef, NodeRef> rel {logic};
    for (auto it1 = order.begin(); it1 != order.end(); ++it1) {
        const Node& src = aeg.lookup(*it1);
        if (!src.is_read()) { continue; }
        for (auto it2 = std::next(it1); it2 != order.end(); ++it2) {
            const Node& dst = aeg.lookup(*it2);
            if (!dst.is_write()) { continue; }
            const z3::expr exists = aeg.fr_exists(*it1, *it2);
            rel.emplace(std::make_tuple(*it1, *it2), eval(exists));
        }
    }
    return rel;
}

template <typename Bool, typename Eval>
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::same_addr() const {
    return binary_rel_if([&] (NodeRef ref1, NodeRef ref2) -> z3::expr {
        const auto& node1 = aeg.lookup(ref1);
        const auto& node2 = aeg.lookup(ref2);
        return node1.arch && node2.arch && node1.same_addr(node2); // TODO: is the arch check redundant?
    });
}

template <typename Bool, typename Eval>
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::same_xstate() const {
    return binary_rel_if([&] (NodeRef ref1, NodeRef ref2) -> z3::expr {
        const auto& node1 = aeg.lookup(ref1);
        const auto& node2 = aeg.lookup(ref2);
        return node1.exec() && node2.exec() && node1.same_xstate(node2); // TODO: is the exec check redundant
    });
}


template <typename Bool, typename Eval>
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::edge_rel(UHBEdge::Kind kind) const {
    relation_type<NodeRef, NodeRef> rel {logic};
    for (NodeRef src : aeg.node_range()) {
        for (NodeRef dst : aeg.node_range()) {
            rel.emplace(std::make_tuple(src, dst), eval(aeg.exists(kind, src, dst)));
        }
    }
    return rel;
}

template <typename Bool, typename Eval>
relation<Bool, NodeRef> Context<Bool, Eval>::node_rel(Inst::Kind kind, ExecMode mode) {
    return node_rel_if([&] (NodeRef, const AEG::Node& node) {
        z3::expr cond {node.arch.ctx()};
        switch (mode) {
            case ARCH: cond = node.arch; break;
            case TRANS: cond = node.trans; break;
            case EXEC: cond = node.exec(); break;
            default: std::abort();
        }
        return cond && node.inst.kind == kind;
    });
}

template <typename Bool, typename Eval>
relation<Bool, NodeRef> Context<Bool, Eval>::node_rel(ExecMode mode) {
    return node_rel_if([&] (NodeRef, const AEG::Node& node) {
        switch (mode) {
            case ARCH: return node.arch;
            case TRANS: return node.trans;
            case EXEC: return node.exec();
            default: std::abort();
        }
    });
}

template <typename Bool, typename T>
T get_singleton(const relation<Bool, T>& a) {
    assert(a.size() == 1);
    return std::get<0>(a.begin()->first);
}

template <typename Bool, typename... Ts>
inline std::ostream& operator<<(std::ostream& os, const relation<Bool, Ts...>& rel) {
    using ::operator<<;
    
    os << "{";
    for (auto it = rel.begin(); it != rel.end(); ++it) {
        if (it != rel.begin()) {
            os << ", ";
        }
        const auto& pair = *it;
        os << "(";
        if constexpr (std::is_same<Bool, bool>()) {
            os << pair.first;
            assert(pair.second);
        } else {
            os << pair;
        }
        os << ")";
    }
    os << "}";
    return os;
}


}
