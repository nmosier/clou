#pragma once

#include <unordered_map>

template <typename Key,
   typename Value,
   typename Hash = std::hash<Key>,
   typename KeyEqual = std::equal_to<Key>
   >
class bimap {
public:
   using map = std::unordered_map<Key, Value, Hash, KeyEqual>;
   using inverse_map = std::unordered_map< 
   
   using key_type = Key;
   using mapped_type = Value;
   using value_type = std::pair<const Key, Value>;
   using size_type = std::size_t;
   using difference_type = std::ptrdiff_t;
   using hasher = Hash;
   using key_equal = KeyEqual;
   using reference = value_type&;
   using const_reference = const value_type&;
   using pointer = value_type *;
   using const_pointer = const value_type *;
   
   
private:
   using unimap = std::unordered_map<
};
