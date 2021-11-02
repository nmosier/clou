#pragma once

#include <vector>
#include <limits>
#include <type_traits>
#include <utility>

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
    }
    
    natural_set& operator=(const natural_set& other) {
        size_ = other.size_;
        v = other.v;
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
            return x;
        }
        
        iterator& operator++() { return inc(); }
        iterator& operator++(int) { return inc(); }
        iterator& operator--() { return dec(); }
        iterator& operator--(int) { return dec(); }
        
        bool operator==(const iterator& other) const { return x == other.x; }
        bool operator!=(const iterator& other) const { return x != other.x; }
        bool operator<(const iterator& other) const { return x < other.x; }
        bool operator<=(const iterator& other) const { return x <= other.x; }
        bool operator>(const iterator& other) const { return x > other.x; }
        bool operator>=(const iterator& other) const { return x >= other.x; }
        
    private:
        Key x;
        const Vec *v;
        
        iterator(Key x, const Vec& v): x(x), v(&v) {
            advance();
        }
        
        void advance() {
            while (x < v->size() && !v->at(x)) {
                ++x;
            }
        }
        
        void retreat() {
            while (x != std::numeric_limits<Key>::max() && !v->at(x)) {
                --x;
            }
        }
        
        iterator& inc() {
            ++x;
            advance();
            return *this;
        }
        
        iterator& dec() {
            --x;
            retreat();
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
        auto ref = v.at(pos.x);
        assert(ref);
        ref = false;
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
        if (v.at(value)) {
            return iterator(value, v);
        } else {
            return end();
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
    
private:
    Vec v;
    size_type size_;
    
    void grow(value_type value) {
        if (v.size() <= value) {
            v.resize(value + 1);
        }
    }
};

}
