#pragma once

#include <utility>
#include <climits>

/* TODO
 * [ ] Use sweep to make clearing simple
 */

template <typename Key, typename T, size_t N, typename Hash = std::hash<Key>,
          typename KeyEqual = std::equal_to<Key>>
class cache_map {
public:
   cache_map(Hash hasher = Hash(), KeyEqual equal = KeyEqual()):
      staleval(1), hasher(hasher), equal(equal) {
      std::fill(stale.begin(), stale.end(), 0);
   }

private:
   using Entry = std::pair<Key, T>;
   using Vec = std::array<Entry, N>;
   unsigned staleval;
   std::array<unsigned, N> stale;
   Vec vec;
   Hash hasher;
   KeyEqual equal;

   size_t index(const Key& key) const {
      return hasher(key) % size();
   }

   const std::pair<const unsigned *, const Entry *> entry(const Key& key) const {
      const size_t idx = index(key);
      return {&stale[idx], &vec[index(key)]};
   }

   std::pair<unsigned *, Entry *> entry(const Key& key) {
      const size_t idx = index(key);
      return {&stale[idx], &vec[index(key)]};
   }

   bool good(const std::pair<const unsigned *, const Entry *>& val) const {
      return *val.first == staleval;
   }

   bool good(const std::pair<unsigned *, Entry *>& val) const {
      return *val.first == staleval;
   }
   
public:
   constexpr size_t size() const { return N; }
   
   const T *read(const Key& key) const {
      const auto& e = entry(key);
      if (good(e) && equal(key, e.second->first)) {
         return &e.second->second;
      } else {
         return nullptr;
      }
   }

   T *read(const Key& key) {
      const auto & e = entry(key);
      if (good(e) && equal(key, e.second->first)) {
         return &e.second->second;
      } else {
         return nullptr;
      }
   }

   void write(const Key& key, const T& val) {
      const auto& e = entry(key);
      *e.second = {key, val};
      *e.first = staleval;
   }
   
#if 0
   void write(const Key& key, T&& val) { entry(key).second = {key, val}; }
   void write(Key&& key, const T& val) { entry(key).second = {key, val}; }
   void write(Key&& key, T&& val) { entry(key).second = {key, val}; }
#endif

   void clear(const Key& key) { *entry(key).first = 0; }
   void clear() {
      ++staleval;
      if (staleval == 0) {
         staleval = 1;
         std::fill(stale.begin(), stale.end(), 0);
      }
   }
};


template <typename T, size_t N, typename Hash = std::hash<T>, typename KeyEqual = std::equal_to<T>>
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

   void clear() { map.clear(); }
   
private:
   cache_map<T, bool, N, Hash, KeyEqual> map;
};
