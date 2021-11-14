#pragma once

#include "aeg/fwd.h"
#include "aeg/constraints.h"

namespace aeg {



struct Edge {
#define AEGEDGE_KIND_X(X)                       \
X(PO)                                        \
X(TFO)                                       \
X(RF)                                        \
X(CO)                                        \
X(FR)                                        \
X(RFX)                                       \
X(COX)                                       \
X(FRX) \
X(ADDR) \
X(ADDR_GEP) \
X(CTRL) \
X(DATA)
    
#define AEGEDGE_KIND_E(name) name,
    enum Kind {
        AEGEDGE_KIND_X(AEGEDGE_KIND_E)
    };
#undef AEGEDGE_KIND_E
    
    Kind kind;
    z3::expr exists;
    Constraints constraints;
    
    static const char *kind_tostr(Kind kind);
    const char *kind_tostr() const { return kind_tostr(kind); }
    static Kind kind_fromstr(const std::string& s);
    
    Edge(Kind kind, Context& ctx, const std::string& name = "");
    
    Edge(Kind kind, const z3::expr& exists): kind(kind), exists(exists) {}
    
    struct Hash {
        size_t operator()(const Edge& x) const { return std::hash<Kind>()(x.kind); }
    };
    
    bool operator==(const Edge& other) const { return kind == other.kind; }
    void simplify() { constraints.simplify(); }
    
    bool possible() const {
        auto e = exists && constraints.get(exists.ctx());
        if constexpr (simplify_before_checking_for_impossible_edges) {
            e = e.simplify();
        }
        return !e.is_false();
    }
};

std::ostream& operator<<(std::ostream& os, const aeg::Edge& e);

inline std::ostream& operator<<(std::ostream& os, aeg::Edge::Kind kind) {
    return os << aeg::Edge::kind_tostr(kind);
}





}
