#pragma once

#include <tuple>
#include <vector>
#include <utility>

#include <z3++.h>

#include "noderef.h"
#include "uhb.h"
#include "aeg/aeg.h"
#include "default_map.h"
#include "util.h"
#include "util/tuple.h"

/** First-order relational logic library
 */
namespace fol {

/* It would be nice to be able to easily construct simple first-order logic expressions/formulas over the AEG
 * to verify that the implementation is correct. For example, we could verify that ~rf.co == fr.
 */

/** Abstract representation of a boolean logic.
 * \tparam Bool The boolean type (bool, z3::expr, etc.) that the logic is over.
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

/** FOL relation, represented as a set of tuples.
 * \tparam Bool boolean type representing element containment
 * \tparam Ts Types of a tuple
 */
template <typename Bool, typename... Ts>
struct relation {
    using Tuple = std::tuple<Ts...>; /*!< Tuples contained in the relation */
    
    /** Hash function for tuples in the relation. */
    struct Hash {
        auto operator()(const Tuple& tuple) const {
            return llvm::hash_value(tuple);
        }
    };
    
    using Map = std::unordered_map<Tuple, Bool, Hash>; /*!< Map from tuples to containment booleans */
    using logic_type = Logic<Bool>;
    using bool_type = Bool;
    
    logic_type logic; /*!< logic structure of this relation */
    Map map;
    
    using const_iterator = typename Map::const_iterator;
    using value_type = typename Map::value_type;
    
    /** Find tuple in relation */
    template <typename... Args>
    const_iterator find(Args&&... args) const { return map.find(std::forward<Args>(args)...); }
    
    const_iterator begin() const { return map.begin(); }
    const_iterator end() const { return map.end(); }
    
    /** Emplace tuple in relation if condition is true. */
    void emplace(const Tuple& tuple, const Bool& cond) {
        if (!logic.is_false(cond)) {
            map.emplace(tuple, cond);
        }
    }
    
    /** Insert tuple in relation if condition is true. */
    void insert(const std::pair<Tuple, Bool>& pair) {
        if (!logic.is_false(pair.second)) {
            map.insert(pair);
        }
    }
    
    /** Insert tuple in relation if condition is true, assigning the existing boolean to `b` if the tuple is already present. */
    void insert_or_assign(const Tuple& t, const Bool& b) {
        if (logic.is_false(b)) {
            map.erase(t);
        } else {
            map.insert_or_assign(t, b);
        }
    }
    
    const Bool& at(const Tuple& t) const { return map.at(t); }
    
    // get by tuple, defaults to false if tuple not in map
    /** Get condition by tuple, defaulting to relation::logic definition of false if tuple not in relation's underlying map. */
    Bool get(const Tuple& t) const {
        const auto it = find(t);
        return it == end() ? logic.F : it->second;
    }
    
    relation(const logic_type& logic = logic_type()): logic(logic) {} /*!< Construct empty relation using logic `logic`. */
    // relation(const relation& other) = default;
    // relation(relation&& other) = default;
};

/** Class for managing necessary information to construct fol::relation's from an ::AEG.
 * \tparam Bool boolean type for tuple containment in relation
 * \tparam Eval functor converting values to `Bool`.
 */
template <typename Bool, typename Eval = util::identity<Bool>>
struct Context {
    using bool_type = Bool;
    using logic_type = Logic<Bool>;
    template <typename... Ts>
    using relation_type = relation<Bool, Ts...>;
    logic_type logic;
    Eval eval;
    AEG& aeg; /*!< AEG to construct relations from */
    
    /** Construct an empty relation given an example tuple. The value type of the relation is equal to the type of `tuple`.
     */
    template <typename... Ts>
    relation_type<Ts...> make_relation(const std::tuple<Ts...>& example_tuple) const {
        return relation_type<Ts...>(logic);
    }
    
    /** Get the binary relation on NodeRef corresponding to the given edge `kind`. */
    relation_type<NodeRef, NodeRef> edge_rel(UHBEdge::Kind kind) const;
    
    relation_type<NodeRef> node_rel(Inst::Kind kind, ExecMode mode); /*!< Get the set of nodes of the given `kind` and execution `mode`. */
    relation_type<NodeRef> node_rel(ExecMode mode); /*!< Get the set of all nodes of the given execution `mode`. */
    
    relation_type<NodeRef> node_rel(z3::expr UHBNode::*pred, ExecMode mode);
    relation_type<NodeRef> node_rel(z3::expr (UHBNode::*pred)() const, ExecMode mode);
    
    /** Get the set of nodes satisfying the condition (of type \a Bool ) when each node is applied to \p pred .
     * \tparam Pred predicate functor type equivalent to std::function<Bool (NodeRef, AEG::NodeRef)>
     * \param pred predicate functor
     * \return Set of nodes
     */
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
    
    /** Get the binary relation over NodeRef consisting of all pairs of nodes that satisy the predicate functor \p pred
     * \tparam Pred Predicate functor type, which should be equivalent to std::function<Bool (NodeRef, NodeRef)>
     * \param pred Predicate functor
     */
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
   
    relation_type<NodeRef, NodeRef> same_addr() const; /*!< Get the same-address binary relation for Context::aeg. It should be reflexive, symmetric, and transitive, except for entry and exit nodes. */
    relation_type<NodeRef, NodeRef> same_xstate() const; /*!< Get the same-xstate binary relation for Context::aeg. It should be reflexive, symmetric, and transitive, except for entry and exit nodes. */
    
    /** Filter the relation \p a by applying the function \p func to each member to produce a new membership condition.
     * \tparam Func Function type, which should be equivalent to std::function<Bool (NodeRef)>.
     * \tparam Ts Relation member types.
     * \param a Input relation
     * \param func Transformation function
     * \returns New relation with filter given by \p func applied
     */
    template <typename Func, typename... Ts>
    relation_type<Ts...> filter(const relation_type<Ts...>& a, Func func) const {
        relation_type<Ts...> res {logic};
        for (const auto& pair : a) {
            res.emplace(pair.first, eval(func(pair.first)) && pair.second);
        }
        return res;
    }
    
    template <typename... Ts>
    relation_type<Ts...> none() const {
        return relation_type<Ts...> {logic};
    }
    
    template <typename... Ts>
    relation_type<Ts...> singleton(const std::tuple<Ts...>& t) const {
        auto rel = none<Ts...>();
        rel.emplace(t, logic.T);
        return rel;
    }
    
    /** Construct a new context for \p AEG over the given boolean logic \p logic and using the evaluator \p eval to convert z3 formulae in \p aeg into values of `Bool` type.
     * \param logic Boolean logic to use
     * \param eval Evaluator to use to convert z3 formulae in \p aeg into `Bool`
     * \param aeg AEG to construct relations from.
     */
    Context(const logic_type& logic, Eval eval, AEG& aeg): logic(logic), eval(eval), aeg(aeg) {}
};

/** Make an empty relation over the given types \p Ts and using the boolean type \p Bool */
template <typename Bool, typename... Ts>
relation<Bool, Ts...> make_relation(const std::tuple<Ts...>& example_tuple, const Logic<Bool>& logic) {
    return relation<Bool, Ts...>(logic);
}

/** Symbolic evaluator, which converts boolean expressions to symbolic z3::expr expressions. */
struct SymEval {
    z3::context *ctx; /*!< underlying context */
    z3::expr operator()(bool b) const { return ctx->bool_val(b); }
    const z3::expr& operator()(const z3::expr& e) const { return e; }
    
    /** \param ctx Underlying context to use */
    SymEval(z3::context& ctx): ctx(&ctx) {}
};

/** Concrete evaluator, which converts boolean expressions to concrete `bool` expressions. */
struct ConEval {
    z3::eval eval; /*!< z3 evalutor use for converting z3 expressions to plain `bool`s */
    bool operator()(bool b) const { return b; }
    bool operator()(const z3::expr& e) const { return (bool) eval(e); }
    
    /** \param eval Z3 evaluator to use */
    ConEval(const z3::eval& eval): eval(eval) {}
};

/** Relational union. Performs \f$ a \gets a \cup b \f$. */
template <typename Bool, typename... Ts>
relation<Bool, Ts...>& operator+=(relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    for (const auto& pair : b) {
        const auto& tuple = pair.first;
        a.insert_or_assign(tuple, a.get(tuple) || pair.second);
    }
    return a;
}

/** Relational union.
 * \return \f$ a \cup b \f$ */
template <typename Bool, typename... Ts>
relation<Bool, Ts...> operator+(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    auto res = a;
    return res += b;
}

/** Relational union. See operator+().
  */
template <typename Bool, typename... Ts>
relation<Bool, Ts...> operator|(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    return a + b;
}

template <typename Bool, typename... Ts>
relation<Bool, Ts...>& operator|=(relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    return a += b;
}

/** Relational difference. Performs \f$ a \gets a \setminus b \f$. */
template <typename Bool, typename... Ts>
relation<Bool, Ts...> operator-=(relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    for (const auto& pair : b) {
        const auto& tuple = pair.first;
        a.insert_or_assign(tuple, a.get(tuple) && !pair.second);
    }
    return a;
}

/** Relational difference.
 * \return \f$ a \setminus b \f$ */
template <typename Bool, typename... Ts>
relation<Bool, Ts...> operator-(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    auto res = a;
    return res -= b;
}

/** Relation intersection.
 * \return \f$ a \cap b \f$ */
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

/** Relation intersection. Performs \f$ a \gets a \cap b \f$. */
template <typename Bool, typename... Ts>
relation<Bool, Ts...>& operator&=(relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    return a = a & b;
}

/** Relational inverse.
 * \return \f$ \left\{ \left(a_n, \ldots, a_1 \right) \mid \left(a_1, \ldots, a_n \right) \in \texttt{in} \right\}  \f$
 */
template <typename Bool, typename... Ts>
auto inverse(const relation<Bool, Ts...>& in) {
    const auto out_tuple = tuples::invert(std::tuple<Ts...>());
    auto out = make_relation(out_tuple, in.logic);
    std::transform(in.begin(), in.end(), std::inserter(out.map, out.map.end()), [] (const auto& in_pair) {
        return std::make_pair(tuples::invert(in_pair.first), in_pair.second);
    });
    return out;
}

/** Relational inverse. See inverse(). */
template <typename Bool, typename... Ts>
auto operator~(const relation<Bool, Ts...>& a) {
    return inverse(a);
}

/** Cartesian product of two relations.
 * \return \f$ a \times b \f$ */
template <typename R1, typename R2>
auto cartesian_product(const R1& a, const R2& b) {
    const auto example_out_tuple = std::tuple_cat(typename R1::Tuple {}, typename R2::Tuple {});
    auto out = make_relation(example_out_tuple, a.logic);
    for (const auto& p1 : a) {
        for (const auto& p2 : b) {
            out.emplace(std::tuple_cat(p1.first, p2.first), p1.second && p2.second);
        }
    }
    return out;
}

template <typename Rel, typename... Rels>
auto generalized_cartesian_product(const Rel& a, Rels&&... rest) {
    if constexpr (sizeof...(rest) == 0) {
        return a;
    } else {
        return cartesian_product(a, generalized_cartesian_product(std::forward<Rels>(rest)...));
    }
}

/** Cartesian product of two relations. See cartesian_product(). */
template <typename R1, typename R2>
auto operator*(const R1& a, const R2& b) {
    return cartesian_product(a, b);
}

/** Inclusive join of two relations.
 * \return \f$  \left\{ \left( a_1, \ldots, a_{n-1}, b_2, \ldots, b_n \right) \mid \left(a_1, \ldots, a_n \right) \in a \wedge \left(b_1, \ldots, b_n \right) \in b \wedge a_n = b_1 \right\} \f$
 */
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
        return std::tuple_cat(tuples::slice<0, Size1-1, Bool>(t1),
                              tuples::slice<1, Size2, Bool>  (t2));
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

/** Restrict a position in a relation to be in a given set.
 * \return \f$  \left\{ (a_1, \ldots, a_n) \mid (a_1, \ldots, a_n) \in a \wedge a_i \in b \right\}\f$, where \a i is equal to \p idx.
 */
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

template <std::size_t begin_idx, std::size_t end_idx, typename Bool, typename T, typename... Ts>
relation<Bool, Ts...> restrict_elements(const relation<Bool, Ts...>& a, const relation<Bool, T>& b) {
    static_assert(end_idx <= sizeof...(Ts));
    if constexpr (begin_idx == end_idx) {
        return a;
    } else {
        return restrict_elements<begin_idx + 1, end_idx>(restrict_element<begin_idx>(a, b), b);
    }
}

template <typename Bool, typename T, typename... Ts>
relation<Bool, Ts...> restrict_elements(const relation<Bool, Ts...>& a, const relation<Bool, T>& b) {
    return restrict_elements<0, sizeof...(Ts)>(a, b);
}

namespace detail {

template <std::size_t I, typename Bool, typename T, typename... Ts>
relation<Bool, Ts...> restrict_any_impl(const relation<Bool, Ts...>& a, const relation<Bool, T>& b) {
    if constexpr (I == sizeof...(Ts)) {
        return relation<Bool, Ts...> {a.logic};
    } else {
        return restrict_any_impl<I+1>(a, b) | restrict_element<I>(a, b);
    }
}

}

template <typename Bool, typename T, typename... Ts>
relation<Bool, Ts...> restrict_any(const relation<Bool, Ts...>& a, const relation<Bool, T>& b) {
    return detail::restrict_any_impl<0>(a, b);
}

// MARK: Predicates

/** Compute whether this relation is empty.
 * \return \f$  a \neq \emptyset \f$ */
template <typename Bool, typename... Ts>
Bool some(const relation<Bool, Ts...>& a) {
    return util::any_of(a.begin(), a.end(), [] (const auto& p) -> Bool {
        return p.second;
    }, a.logic.F);
}

/** Compute whether this relation has exactly one element.
 * \return \f$ \left| a \right| = 1 \f$ */
template <typename Bool, typename... Ts>
Bool one(const relation<Bool, Ts...>& a) {
    if constexpr (std::is_same<Bool, z3::expr>()) {
        z3::expr_vector vec {a.logic.T.ctx()};
        for (const auto& pair : a) {
            vec.push_back(pair.second);
        }
        return z3::exactly(vec, 1);
    } else {
        return util::one_of(a.begin(), a.end(), [] (const auto& p) -> Bool { return p.second; }, a.logic.T, a.logic.F);
    }
}

/** Compute whether this relation is empty.
 * \return \f$ a = \emptyset \f$ */
template <typename Bool, typename... Ts>
Bool no(const relation<Bool, Ts...>& a) {
    return !some(a);
}

/** Compute whether this relation has one or zero elements.
 * \return \f$ \left| a \right| \leq 1 \f$ */
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

/** Compute whether \p a is a subset of \p b
 * \return \f$ a \subseteq b \f$
 */
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

/** Compute whether two relations are equal.
 * \return \f$ a = b \f$ */
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

/** Compute whether two relations are equal. See equal(). */
template <typename Bool, typename... Ts>
Bool operator==(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    return equal(a, b);
}

/** Compute whether two relations are not equal
 * \return \f$ a \neq b \f$ */
template <typename Bool, typename... Ts>
Bool not_equal(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    return !equal(a, b);
}

/** Compute whether two relations are not equal. See not_equal(). */
template <typename Bool, typename... Ts>
Bool operator!=(const relation<Bool, Ts...>& a, const relation<Bool, Ts...>& b) {
    return not_equal(a, b);
}

/** Extract the set consisting of the elements at the given position in the relation.
 * \tparam N Element position to extract.
 * \return \f$  \left\{ a_N \mid (a_1, \ldots, a_n) \in a \right\} \f$ */
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

/** Compute the irreflexive transitive closure of the binary relation \p a.
 * \tparam T the type of both elements in the binary relation \p a.
 * \return \f$   a^* \text{ such that }  \forall (a_1, a_3) \in a^*, \left( \exists a_2 \mid (a_1, a_2), (a_2, a_3) \in a^* \right) \vee (a_1, a_3) \in a \f$
 */
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

/** Get the \p N-ary identity relation over set \p a
 * \return \f$ \left\{ \{x\}^N \mid x \in a \right\} \f$ */
template <std::size_t N = 2, typename Bool, typename T>
auto identity(const relation<Bool, T>& a) {
    auto res = make_relation(tuples::make_repeated<N, T>(T()), a.logic);
    for (const auto& pair : a) {
        res.emplace(tuples::make_repeated<N>(std::get<0>(pair.first)), pair.second);
    }
    return res;
}

/** Duplicate the last element of the relation.
 * \return \f$  \left\{ (a_1, \ldots, a_n, a_n) \mid (a_1, \ldots, a_n) \in a \f$
 */
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

/** Inclusive join.
 * \return \f$ \left\{ (a_1, \ldots, a_n, b_2, \ldots, b_m) \mid (a_1, \ldots, a_n) \in a \wedge (b_1, \ldots, b_m) \in b \wedge a_n = b_1  \right\} \f$ */
template <typename Rel1, typename Rel2>
auto join2(const Rel1& a_, const Rel2& b) {
    const auto a = extend_last(a_);
    return join(a, b);
}

/** Check whether the relation is cylic. */
template <typename Bool, typename T>
Bool cyclic(const relation<T,T>& a) {
    const relation<Bool, T> srcs = element<0>(a);
    const relation<Bool, T, T> id = identity(srcs);
    const relation<Bool, T, T> closure = irreflexive_transitive_closure(a);
    return some(id & closure);
}

/** Check whether the relation is acyclic. */
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
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::same_addr() const {
    return binary_rel_if([&] (NodeRef ref1, NodeRef ref2) -> z3::expr {
        const auto& node1 = aeg.lookup(ref1);
        const auto& node2 = aeg.lookup(ref2);
        return node1.arch && node2.arch && node1.same_addr(node2); // TODO: is the arch check redundant?
    });
}

template <typename Bool, typename Eval>
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::same_xstate() const {
    relation_type<NodeRef, NodeRef> rel;
    const auto skip = [&] (NodeRef ref) -> bool {
        return logic.is_false(eval(aeg.lookup(ref).xsaccess()));
    };
    for (NodeRef src : aeg.node_range()) {
        if (skip(src)) { continue; }
        for (NodeRef dst : aeg.node_range()) {
            if (skip(dst)) { continue; }
            rel.emplace(std::make_tuple(src, dst), aeg.lookup(src).same_xstate(aeg.lookup(dst)));
        }
    }
    return rel;
}

template <typename Bool, typename Eval>
relation<Bool, NodeRef, NodeRef> Context<Bool, Eval>::edge_rel(UHBEdge::Kind kind) const {
    relation_type<NodeRef, NodeRef> rel {logic};
    for (const NodeRef src : aeg.node_range()) {
        if (logic.is_false(eval(aeg.exists_src(kind, src)))) { continue; }
        for (const NodeRef dst : aeg.node_range()) {
            if (logic.is_false(eval(aeg.exists_dst(kind, dst)))) { continue; }
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
        return cond && node.inst->kind() == kind;
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

template <typename Bool, typename Eval>
relation<Bool, NodeRef> Context<Bool, Eval>::node_rel(z3::expr UHBNode::*pred, ExecMode mode) {
    return node_rel_if([&] (NodeRef, const AEG::Node& node) {
        z3::context& ctx = node.arch.ctx();
        z3::expr cond {ctx};
        // TODO: unify this switch with identical above switch
        switch (mode) {
            case ARCH: cond = node.arch; break;
            case TRANS: cond = node.trans; break;
            case EXEC: cond = node.exec(); break;
            default: std::abort();
        }
        return cond && node.*pred;
    });
}

template <typename Bool, typename Eval>
relation<Bool, NodeRef> Context<Bool, Eval>::node_rel(z3::expr (UHBNode::*pred)() const, ExecMode mode) {
    return node_rel_if([&] (NodeRef, const AEG::Node& node) {
        z3::context& ctx = node.arch.ctx();
        z3::expr cond {ctx};
        // TODO: unify this switch with identical above switch
        switch (mode) {
            case ARCH: cond = node.arch; break;
            case TRANS: cond = node.trans; break;
            case EXEC: cond = node.exec(); break;
            default: std::abort();
        }
        return cond && (node.*pred)();
    });
}

template <typename T>
T get_singleton(const relation<bool, T>& a) {
    assert(a.map.size() == 1);
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

namespace detail {

template <typename Bool, typename T>
struct Singleton {
    T value;
    relation<Bool, T> rel;
};

template <typename Pred, typename Tuple, typename Rel>
void set_comprehension_impl(Pred pred, const Tuple& tuple, Rel& rel) {
    // get condition
    auto cond = std::apply(pred, tuple);
    
    // transform back to regular tuple
    const auto entry = tuples::transform(tuple, [&] (const auto& x) {
        assert(x.map.size() == 1);
        const auto it = x.begin();
        cond = cond && it->second;
        return std::get<0>(it->first);
    });
    
    rel.emplace(entry, cond);
}

template <typename Pred, typename Tuple, typename Rel, typename Set, typename... Sets>
void set_comprehension_impl(Pred pred, const Tuple& tuple, Rel& rel, const Set& bag, Sets&&... bags) {
    for (const auto& pair : bag) {
        auto singleton = make_relation(pair.first, rel.logic);
        singleton.insert(pair);
        const auto newtuple = std::tuple_cat(tuple, std::make_tuple(singleton));
        set_comprehension_impl(pred, newtuple, rel, std::forward<Sets>(bags)...);
    }
}

template <typename Pred, typename Set, typename... Sets>
auto set_comprehension_impl0(Pred pred, const Set& set, Sets&&... sets) {
    // TODO
    const auto& logic = set.logic;
    decltype(generalized_cartesian_product(set, std::forward<Sets>(sets)...)) rel {logic};
    set_comprehension_impl(pred, std::make_tuple(), rel, set, std::forward<Sets>(sets)...);
    return rel;
}

}

template <typename Pred, typename... Sets>
auto set_comprehension(Pred pred, Sets&&... sets) {
    return detail::set_comprehension_impl0(pred, std::forward<Sets>(sets)...);
}

}
