#include "uhb.h"
#include "config.h"

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

UHBEdge::Kind UHBEdge::kind_fromstr(const std::string& s_) {
#define UHBEDGE_KIND_PAIR(name) {#name, name},
    static const std::unordered_map<std::string, Kind> map {
        UHBEDGE_KIND_X(UHBEDGE_KIND_PAIR)
    };
    std::string s;
    std::transform(s_.begin(), s_.end(), std::back_inserter(s), toupper);
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
#if 1
UHBNode::UHBNode(const Inst& inst, UHBContext& c): inst(inst), arch(c.context), trans(c.context), trans_depth(c.context), xsread(c.context), xswrite(c.context), exec_order(c.context), trans_group_min(c.context), trans_group_max(c.context), xsread_order(c.context), xswrite_order(c.context), constraints() {}
#else
UHBNode::UHBNode(const Inst& inst, UHBContext& c):
inst(inst), arch(c.make_bool("arch")),
trans(c.make_bool("trans")), trans_depth(c.make_int("depth")),
xsread(c.FALSE), xswrite(c.FALSE),
constraints() {}
#endif

UHBContext::UHBContext(): context(), TRUE(context.bool_val(true)), FALSE(context.bool_val(false)) {}

void UHBConstraints::add_to(z3::solver& solver) const {
   for (const auto& p : exprs) {
      std::stringstream ss;
       if (include_expr_in_constraint_name) {
           ss << p.first << ":";
       }
      ss << p.second << ":" << constraint_counter++;
      try {
         solver.add(p.first, ss.str().c_str());
      } catch (const z3::exception& e) {
         logv(0) << e.msg() << "\n";
         std::abort();
      }
   }
}

void UHBConstraints::operator()(const z3::expr& clause, const std::string& name) {
    if ((simplify_before_checking_for_false_constraints ? clause.simplify() : clause).is_false()) {
       std::cerr << "adding false constraint: " << clause << "\n";
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

bool UHBNode::is_special() const {
    switch (inst.kind) {
        case Inst::ENTRY:
        case Inst::EXIT:
            return true;
        case Inst::READ:
        case Inst::WRITE:
            return false;
        default:
            throw std::logic_error("same addr called on a non-memory-access");
    }
}

z3::expr UHBNode::same_addr(const UHBNode& a, const UHBNode& b) {
    if (a.is_special() || b.is_special()) {
        return a.arch.ctx().bool_val(true);
    } else {
        return a.get_memory_address() == b.get_memory_address();
    }
}
