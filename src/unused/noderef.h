#pragma once

#include <utility>

#if 0
template <typename Class>
class NodeRef {
public:
   unsigned idx;
   operator unsigned() const { return idx; }
   NodeRef() {}
   NodeRef(unsigned idx): idx(idx) {}

   struct Index {
      size_t operator()(const NodeRef& ref) const { return static_cast<unsigned>(ref); }
   };
   
   NodeRef& operator++() { ++idx; return *this; }

private:
   friend std::hash<NodeRef<Class>>;
};

namespace std {
   template <typename Class>
   struct hash<NodeRef<Class>> {
      std::size_t operator()(const NodeRef<Class>& ref) const {
         return std::hash<unsigned>()(ref.idx);
      }
   };
}
#endif
