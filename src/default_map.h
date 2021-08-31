#pragma once

#include <unordered_map>
#include <utility>

#if 0
template <typename Key, typename T, typename Hash = std::hash<Key>, typename KeyEqual = std::equal_to<Key>>
class default_map: public std::unordered_map<Key, T, Hash, KeyEqual> {
public:
    template <typename... Args>
    default_map(const T& default_value, Args&&... args): std::unordered_map(std::forward<Args>(args)...), default_value(default_value) {}
    
    T& operator[](const Key& key) {
        auto it = find(key);
        if (it == end()) {
            it = emplace(key, default_value).first;
        }
        return it->second;
    }
    
    T& operator[](Key&& key) {
        auto it = find(key);
        if (it == end()) {
            it = emplace(key, default_value).first;
        }
        return it->second;
    }
    
private:
    T default_value;
};
#endif
