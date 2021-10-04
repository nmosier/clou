#include "uhb.h"
#include "config.h"
#include "aeg.h"

unsigned constraint_counter = 0;

std::ostream& operator<<(std::ostream& os, const UHBEdge& e) {
    os << e.kind << " " << e.exists << "\n"
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
UHBNode::UHBNode(const Inst& inst, UHBContext& c): inst(inst), arch(c), trans(c), trans_depth(c), introduces_trans(c), xsread(c), xswrite(c), arch_order(c), exec_order(c), trans_group_min(c), trans_group_max(c), mem(c), taint(c), taint_mem(c), constraints() {}

UHBContext::UHBContext(): context(), TRUE(context.bool_val(true)), FALSE(context.bool_val(false)) {}

void UHBConstraints::add_to(z3::solver& solver) const {
   for (const auto& p : exprs) {
      std::stringstream ss;
       if (include_expr_in_constraint_name) {
           ss << p.first << ":";
       }
      ss << p.second << ":" << constraint_counter++;
      try {
          if constexpr (should_name_constraints) {
              solver.add(p.first, ss.str().c_str());
          } else {
              solver.add(p.first);
          }
      } catch (const z3::exception& e) {
         logv(0) << e.msg() << "\n";
         std::abort();
      }
   }
}

void UHBConstraints::operator()(const z3::expr& clause, const std::string& name) {
    assert(!name.empty());
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

bool UHBNode::is_special() const {
    switch (inst.kind) {
        case Inst::ENTRY:
        case Inst::EXIT:
            return true;
        default:
            return false;
    }
}

z3::expr UHBNode::same_addr(const UHBNode& a, const UHBNode& b) {
    z3::context& ctx = a.arch.ctx();
    if (a.is_special() || b.is_special()) {
        return ctx.bool_val(true);
    }
    if (!(a.is_memory_op() && b.is_memory_op())) {
        return ctx.bool_val(false);
    }
    return a.get_memory_address() == b.get_memory_address();
}

z3::expr UHBNode::same_xstate(const UHBNode& a, const UHBNode& b) {
    z3::context& ctx = a.arch.ctx();
    if (a.is_special() || b.is_special()) {
        return ctx.bool_val(true);
    }
    if (a.xsaccess().is_false() || b.xsaccess().is_false()) {
        return ctx.bool_val(false);
    }
    return *a.xstate == *b.xstate;
}

z3::expr UHBNode::xsaccess_order_less::operator()(NodeRef a, NodeRef b) const {
    if (a == b) { return aeg.context.FALSE; }
   const UHBNode& an = aeg.lookup(a);
   const UHBNode& bn = aeg.lookup(b);
    
    const auto is_entry = [&] (NodeRef ref) -> bool { return ref == aeg.entry; };
    const auto is_exit = [&] (NodeRef ref) -> bool { return aeg.exits.find(ref) != aeg.exits.end(); };

    if (is_entry(a) && is_entry(b)) { return aeg.context.FALSE; }
    if (is_entry(a) && is_exit(b))  { return aeg.context.TRUE; }
    if (is_exit(a) && is_entry(b))  { return aeg.context.FALSE; }
    if (is_exit(a) && is_exit(b))   { return aeg.context.bool_val(a < b); } // NOTE: this is consistent with ordering for non-special nodes.
    if (is_entry(a)) { return aeg.context.TRUE; }
    if (is_entry(b)) { return aeg.context.FALSE; }
    if (is_exit(a))  { return aeg.context.FALSE; }
    if (is_exit(b))  { return aeg.context.TRUE; }
    
    return a < b ? *an.xsaccess_order <= *bn.xsaccess_order : *an.xsaccess_order < *bn.xsaccess_order;
}

bool UHBNode::access_order_less::operator()(NodeRef a, NodeRef b) const {
    // TODO: imporve runtime
    NodeRefVec order;
    aeg.po.reverse_postorder(std::back_inserter(order));
    const auto a_it = std::find(order.begin(), order.end(), a);
    const auto b_it = std::find(order.begin(), order.end(), b);
    return a_it < b_it;
}
