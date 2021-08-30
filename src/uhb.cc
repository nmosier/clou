#include "uhb.h"

unsigned constraint_counter = 0;

std::ostream& operator<<(std::ostream& os, const UHBEdge& e) {
    os << e.kind_tostr() << " " << e.exists << "\n"
    << e.constraints << "\n";
    return os;
}

const char *UHBEdge::kind_tostr(Kind kind) {
#define UHBEDGE_KIND_CASE(name) case name: return #name;
    switch (kind) {
            UHBEDGE_KIND_X(UHBEDGE_KIND_CASE)
        default: return nullptr;
    }
#undef UHBEDGE_KIND_CASE
}

UHBEdge::Kind UHBEdge::kind_fromstr(const std::string& s) {
#define UHBEDGE_KIND_PAIR(name) {#name, name},
    static const std::unordered_map<std::string, Kind> map {
        UHBEDGE_KIND_X(UHBEDGE_KIND_PAIR)
    };
    return map.at(s);
#undef UHBEDGE_KIND_PAIR
}

/* PO
 * Create a fresh bool. Predecessors imply exactly one successor's bool is true.
 *
 * TFO
 * Create a fresh bool. Predecessors imply any number of successor's bool is true.
 *
 * po => tfo.
 *
 * TFO must be at most n hops away from a po.
 * OR all nodes exactly distance n away together (or TOP, in the edge case).
 */
UHBNode::UHBNode(const Inst& inst, UHBContext& c):
inst(inst), arch(c.make_bool("arch")),
trans(c.make_bool("trans")), trans_depth(c.make_int("depth")),
xsread(c.FALSE), xswrite(c.FALSE),
constraints() {}

UHBContext::UHBContext(): context(), TRUE(context.bool_val(true)), FALSE(context.bool_val(false)) {}

void UHBConstraints::add_to(z3::solver& solver) const {
   for (const auto& p : exprs) {
      std::stringstream ss;
      ss << p.first << ":" << p.second << ":" << constraint_counter++;
      try {
         solver.add(p.first, ss.str().c_str());
      } catch (const z3::exception& e) {
         logv(0) << e.what() << "\n";
         std::abort();
      }
   }
}

void UHBConstraints::operator()(const z3::expr& clause, const std::string& name) {
   if (clause.simplify().is_false()) {
      throw std::logic_error("adding constraint 'false'");
   }
   exprs.emplace_back(clause, name);
}

void UHBConstraints::simplify() {
    std::for_each(exprs.begin(), exprs.end(), [] (auto& p) {
        z3::expr& e = p.first;
        e = e.simplify();
    });
}

void UHBNode::simplify() {
    arch = arch.simplify();
    trans = trans.simplify();
    trans_depth = trans_depth.simplify();
    constraints.simplify();
}

z3::expr UHBNode::get_xsaccess(XSAccess kind) const {
    switch (kind) {
        case XSAccess::XSREAD: return xsread;
        case XSAccess::XSWRITE: return xswrite;
        default: std::abort();
    }
}

z3::expr UHBNode::same_addr(const UHBNode& a, const UHBNode& b) {
    using K = Inst::Kind;
    
    const auto is_special = [] (K kind) -> bool {
        switch (kind) {
            case K::ENTRY:
            case K::EXIT:
                return true;
            case K::READ:
            case K::WRITE:
                return false;
            default:
                throw std::logic_error("same addr called on a non-memory-access");
        }
    };
    
    if (is_special(a.inst.kind) || is_special(b.inst.kind)) {
        return a.arch.ctx().bool_val(true);
    } else {
        return a.get_addr_ref(0) == b.get_addr_ref(0);
    }
}
