#include "noderef.h"

namespace {

template <class Key, class T, class Hash, class KeyEqual, class Allocator, class Combine>
std::unordered_map<Key, T, Hash, KeyEqual, Allocator>& update_assign(std::unordered_map<Key, T, Hash, KeyEqual, Allocator>& a, const std::unordered_map<Key, T, Hash, KeyEqual, Allocator>& b, Combine combine) {
    for (const auto& bp : b) {
        combine(a[bp.first], bp.second);
    }
    return a;
}

}

NodeRefMap& operator+=(NodeRefMap& a, const NodeRefMap& b) {
    if (a.empty()) {
        a = b;
        return a;
    } else {
        return update_assign(a, b, [] (NodeRefSet& a, const NodeRefSet& b) {
            std::copy(b.begin(), b.end(), std::inserter(a, a.end()));
        });
    }
}
