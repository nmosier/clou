#include "aeg/edge.h"
#include "aeg/context.h"

namespace aeg {


Edge::Edge(Kind kind, Context& ctx, const std::string& name): kind(kind), exists(ctx.make_bool(name)) {}


std::ostream& operator<<(std::ostream& os, const aeg::Edge& e) {
    os << e.kind << " " << e.exists << "\n"
    << e.constraints << "\n";
    return os;
}

unsigned constraint_counter = 0;

const char *Edge::kind_tostr(Kind kind) {
#define AEGEDGE_KIND_CASE(name) case name: return #name;
    switch (kind) {
            AEGEDGE_KIND_X(AEGEDGE_KIND_CASE)
        default: return nullptr;
    }
#undef AEGEDGE_KIND_CASE
}

Edge::Kind Edge::kind_fromstr(const std::string& s_) {
#define AEGEDGE_KIND_PAIR(name) {#name, name},
    static const std::unordered_map<std::string, Kind> map {
        AEGEDGE_KIND_X(AEGEDGE_KIND_PAIR)
    };
#undef AEGEDGE_KIND_PAIR
    std::string s;
    std::transform(s_.begin(), s_.end(), std::back_inserter(s), toupper);
    
    const auto it = map.find(s);
    if (it == map.end()) {
        throw std::invalid_argument("string is not edge kind");
    } else {
        return it->second;
    }
}


}
