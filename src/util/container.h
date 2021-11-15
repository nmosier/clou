#pragma once

#include <vector>
#include <limits>
#include <type_traits>
#include <utility>
#include <numeric>
#include <optional>
#include <algorithm>
#include <memory>
#include <set>
#include <unordered_map>
#include <iostream>

#include "functional.h"

namespace util {


/** Set of natural (unsigned) integers. Best use case is when dense.
 */
template <typename Key>
class natural_set {
    static_assert(std::is_unsigned<Key>(), "Key to natural_set must be unsigned integral type");
    using Vec = std::vector<bool>;
public:
    using key_type = Key;
    using value_type = Key;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = value_type;
    using const_reference = value_type;
    class iterator;
    using const_iterator = iterator;
    
    /* CONSTRUCTORS */
    
    natural_set(): size_(0) {}
    
    template <typename InputIt>
    natural_set(InputIt first, InputIt last): size_(0) {
        insert(first, last);
    }
    
    natural_set(const natural_set& other) {
        *this = other;
    }
    
    natural_set(natural_set&& other) {
        *this = other;
    }
    
    natural_set(std::initializer_list<value_type> init): size_(0) {
        insert(init);
    }
    
    natural_set& operator=(natural_set&& other) {
        size_ = other.size_; other.size_ = 0;
        v = std::move(other.v);
        return *this;
    }
    
    natural_set& operator=(const natural_set& other) {
        size_ = other.size_;
        v = other.v;
        return *this;
    }
    
    /* ITERATORS */
    
    class iterator {
    public:
        /* ITERATOR TRAITS */
        using difference_type = std::ptrdiff_t;
        using value_type = natural_set::value_type;
        using pointer = void;
        using reference = value_type;
        using iterator_category = std::bidirectional_iterator_tag;
        
        iterator() {}
        
        value_type operator*() const {
            return *x;
        }
        
        iterator& operator++() { return inc(); }
        iterator& operator++(int) { return inc(); }
        
        bool operator==(const iterator& other) const { return x == other.x; }
        bool operator!=(const iterator& other) const { return x != other.x; }
        bool operator<(const iterator& other) const { return x < other.x; }
        bool operator<=(const iterator& other) const { return x <= other.x; }
        bool operator>(const iterator& other) const { return x > other.x; }
        bool operator>=(const iterator& other) const { return x >= other.x; }
        
    private:
        std::optional<Key> x;
        const Vec *v;
        
        iterator(Key x, const Vec& v): x(x), v(&v) {
            advance();
        }
        
        void advance() {
            if (x) {
                const auto it = std::find(v->begin() + *x, v->end(), true);
                x = it - v->begin();
            }
            if (x >= v->size()) {
                x = std::nullopt;
            }
        }
        
        iterator& inc() {
            assert(x);
            ++*x;
            advance();
            return *this;
        }
        
        friend natural_set;
    };
    
    iterator begin() const noexcept {
        return iterator(0, v);
    }

    iterator end() const noexcept {
        return iterator(v.size(), v);
    }
    
    /* CAPACITY */
    
    bool empty() const noexcept {
        return size_ == 0;
    }
    
    size_type size() const noexcept {
        return size_;
    }
    
    size_type max_size() const noexcept {
        return std::numeric_limits<Key>::max();
    }
    
    /* MODIFIERS */
    
    void clear() noexcept {
        v.clear();
        size_ = 0;
    }
    
    std::pair<iterator, bool> insert(value_type value) {
        grow(value);
        auto ref = v.at(value);
        std::pair<iterator, bool> res;
        res.second = ref;
        if (!ref) {
            ++size_;
        }
        ref = true;
        res.first = iterator(value, v);
        return res;
    }
    
    iterator insert(const_iterator hint, value_type value) {
        return insert(value);
    }
    
    template <typename InputIt>
    void insert(InputIt first, InputIt last) {
        for (auto it = first; it != last; ++it) {
            insert(*it);
        }
    }
    
    void insert(std::initializer_list<value_type> ilist) {
        insert(ilist.begin(), ilist.end());
    }
    
    std::pair<iterator, bool> emplace(value_type value) {
        return insert(value);
    }
    
    std::pair<iterator, bool> emplace_hint(const_iterator hint, value_type value) {
        return insert(hint, value);
    }
    
    iterator erase(iterator pos) {
        auto ref = v.at(*pos.x);
        assert(ref);
        ref = false;
        assert(!v.at(*pos.x));
        --size_;
        pos.advance();
        return pos;
    }
    
    /* iterator erase(const_iterator pos); */
    
    iterator erase(const_iterator first, const_iterator last) {
        for (auto it = first; it != last; ++it) {
            erase(it);
        }
        return last;
    }
    
    size_type erase(value_type value) {
        auto ref = v.at(value);
        if (ref) {
            --size_;
            ref = false;
            return 1;
        } else {
            return 0;
        }
    }
    
    void swap(natural_set& other) {
        std::swap(v, other.v);
        std::swap(size_, other.size_);
    }
    
    /* void merge(natural_set& other); */
    
    /* LOOKUP */
    
    size_type count(value_type value) const {
        return v.at(value) ? 1 : 0;
    }

    iterator find(value_type value) const {
        if (value >= v.size()) {
            return end();
        } else if (v.at(value)) {
            return iterator(value, v);
        } else {
            return end();
        }
    }
    
    bool contains(value_type value) const noexcept {
        if (value >= v.size()) {
            return false;
        } else {
            return v.at(value);
        }
    }
    
    /* COMPARISONS */
    
    bool operator==(const natural_set& other) const {
        if (size() != other.size()) {
            return false;
        }
        const auto v_size = std::min(v.size(), other.v.size());
        return std::equal(v.begin(), v.begin() + v_size, other.v.begin());
    }
    
    bool operator!=(const natural_set& other) const {
        return !(*this == other);
    }
    
    /* OPERATORS */
    
    natural_set& operator&=(const natural_set& other) {
        const auto count = std::min(v.size(), other.v.size());
        v.resize(count);
        std::transform(v.begin(), v.end(), other.v.begin(), v.begin(), std::logical_and());
        recompute_size();
        return *this;
    }
    
    natural_set& operator|=(const natural_set& other) {
        const auto count = std::max(v.size(), other.v.size());
        v.resize(count);
        std::transform(v.begin(), v.begin() + other.v.size(), other.v.begin(), v.begin(), std::logical_or());
        recompute_size();
        return *this;
    }
    
private:
    Vec v;
    size_type size_;
    
    void grow(value_type value) {
        if (v.size() <= value) {
            v.resize(value + 1);
        }
    }
    
    void recompute_size() {
        size_ = std::reduce(v.begin(), v.end());
    }
};


template <class Key, class T, class Hash = std::hash<Key>, class KeyEqual = std::equal_to<Key>>
class table_map {
    using Table = std::set<T>;
    using TableIt = typename Table::const_iterator;
    using Map = std::unordered_map<Key, TableIt, Hash, KeyEqual>;
public:
    using key_type = Key;
    using mapped_type = T;
    using value_type = std::pair<const Key, T>;
    using size_type = typename Map::size_type;
    using difference_type = typename Map::difference_type;
    using hasher = Hash;
    using key_equal = KeyEqual;
    using reference = std::pair<const Key&, const T&>;
    using const_reference = reference;
    // using pointer = ...
    // using const_pointer = ...
    
    class iterator {
        using MapIt = typename Map::const_iterator;
    public:
        /* ITERATOR TRAITS */
        using difference_type = std::ptrdiff_t;
        using value_type = table_map::value_type;
        using pointer = void;
        using reference = table_map::reference;
        using iterator_category = std::bidirectional_iterator_tag;
        
        iterator() {}
        
        reference operator*() const {
            return reference {it->first, *it->second};
        }
        
        
        
        iterator& operator++() { ++it; return *this; }
        iterator& operator++(int) { it++; return *this; }
        iterator& operator--() { --it; return *this; }
        iterator& operator--(int) { it--; return *this; }
        
#if 0
        auto operator<=>(const iterator& o) const { return it <=> o.it; }
#else
        auto operator<=>(const iterator&) const = default;
        bool operator==(const iterator&) const = default;
#endif
        
    private:
        MapIt it;
        
        iterator(MapIt it): it(it) {}
        
        friend table_map;
    };
    
    using const_iterator = iterator;
    
    /* ITERATORS */
    iterator begin() const noexcept { return iterator(map.begin()); }
    iterator end() const noexcept { return iterator(map.end()); }
    
    /* CAPACITY */
    bool empty() const noexcept { return map.empty(); }
    size_type size() const noexcept { return map.size(); }
    size_type max_size() const noexcept { return map.max_size(); }
    
    /* MODIFIERS */
    void clear() noexcept { map.clear(); }
    
    std::pair<iterator, bool> insert(const reference& value) {
        /* insert into table */
        const auto table_it = table->insert(value.second).first;
        const auto map_res = map.emplace(value.first, table_it);
        return std::make_pair(iterator(map_res.first), map_res.second);
    }
    
#if 0
    template <class InputIt>
    void insert(InputIt first, InputIt last) {
        for (auto it = first; it != last; ++it) {
            insert(*it);
        }
    }
    
    void insert(std::initializer_list<value_type> ilist) {
        insert(ilist.begin(), ilist.end());
    }
#endif
    
    std::pair<iterator, bool> insert_or_assign(const Key& key, const T& value) {
        const auto table_it = table->insert(value).first;
        const auto map_res = map.insert_or_assign(key, table_it);
        return std::make_pair(iterator(map_res.first), map_res.second);
    }
    
    std::pair<iterator, bool> emplace(const Key& key, const T& value) {
        const auto table_it = table->insert(value).first;
        const auto map_res = map.emplace(key, table_it);
        return std::make_pair(iterator(map_res.first), map_res.second);
    }
    
    template <typename Mod>
    void update(const Key& key, Mod mod) {
        T value = operator[](key);
        mod(value);
        insert_or_assign(key, value); // TODO: should only be assign().
    }
    
    iterator erase(iterator pos) {
        return iterator(map.erase(pos.it));
    }
    
    size_type erase(const Key& key) {
        return map.erase(key);
    }
    
    /* LOOKUP */
    
    const T& at(const Key& key) const { return *map.at(key); }
    
    const T& operator[](const Key& key) {
        const auto table_it = table->insert(T()).first;
        const auto map_res = map.emplace(key, table_it);
        const auto map_it = map_res.first;
        return *map_it->second;
    }
    
    size_type count(const Key& key) const { return map.count(key); }
    iterator find(const Key& key) const { return iterator(map.find(key)); }
    bool contains(const Key& key) const { return map.contains(key); }
    bool operator==(const table_map& other) const { return map == other.map; }
    bool operator!=(const table_map& other) const { return !(*this == other); }
    
    /* CONSTRUCTORS */
    table_map(Table& table): table(&table) {}
    table_map(Table& table, size_type size): table(&table), map(size) {}
    
private:
    Table *table;
    Map map;
    
};


}
