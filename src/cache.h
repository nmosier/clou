#pragma once

#include <utility>

template <typename Key, typename T, typename Hash = std::hash<Key>,
          typename KeyEqual = std::equal_to<Key>>
class cache_map {
public:
   cache_map(size_t size = 0x1000, Hash hasher = Hash(), KeyEqual equal = KeyEqual()):
      vec(size), hasher(hasher), equal(equal) {}

private:
   using Entry = std::optional<std::pair<Key, T>>;
   using Vec = std::vector<Entry>;
   Vec vec;
   Hash hasher;
   KeyEqual equal;

   size_t index(const Key& key) const {
      return hasher(key) % size();
   }

   const Entry& entry(const Key& key) const {
      return vec[index(key)];
   }

   Entry& entry(const Key& key) {
      return vec[index(key)];
   }
   
public:
   size_t size() const { return vec.size(); }
   
   const T *read(const Key& key) const {
      const Entry& e = entry(key);
      if (e && equal(key, e->first)) {
         return &e->second;
      } else {
         return nullptr;
      }
   }

   T *read(const Key& key) {
      Entry& e = entry(key);
      if (e && equal(key, e->first)) {
         return &e->second;
      } else {
         return nullptr;
      }
   }

   void write(const Key& key, const T& val) { entry(key) = {key, val}; }
   void write(const Key& key, T&& val) { entry(key) = {key, val}; }
   void write(Key&& key, const T& val) { entry(key) = {key, val}; }
   void write(Key&& key, T&& val) { entry(key) = {key, val}; }

   void clear(const Key& key) { entry(key) = std::nullopt; }
};


template <typename T, typename Hash = std::hash<T>, typename KeyEqual = std::equal_to<T>>
class cache_set {
public:
   template <typename... Args>
   cache_set(Args&&... args): map(std::forward<Args>(args)...) {}

   size_t size() const { return map.size(); }

   enum Answer {
      YES, NO, MAYBE,
   };
   
   Answer contains(const T& val) const {
      const bool *p = map.read(val);
      if (p) {
         if (*p) {
            return YES;
         } else {
            return NO;
         }
      } else {
         return MAYBE;
      }
   }

   void insert(const T& val) { map.write(val, true); }
   void erase(const T& val) { map.write(val, false); }
   
private:
   cache_map<T, bool, Hash, KeyEqual> map;
};
