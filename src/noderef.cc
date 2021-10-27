#include "noderef.h"
#include "util.h"

NodeRefMap& operator+=(NodeRefMap& a, const NodeRefMap& b) {
    if (a.empty()) {
        a = b;
        return a;
    } else {
        return util::update_assign(a, b, [] (NodeRefSet& a, const NodeRefSet& b) {
            std::copy(b.begin(), b.end(), std::inserter(a, a.end()));
        });
    }
}
