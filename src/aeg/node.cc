#include "aeg/node.h"
#include "inst.h"
#include "aeg/context.h"
#include "aeg/aeg.h"
#include "cfg/expanded.h"

namespace aeg {

Node::Node(std::unique_ptr<Inst>&& inst, Context& c): inst(std::move(inst)), arch(c), trans(c), read(c), write(c), xsread(c), xswrite(c), constraints() {}

bool Node::may_read() const {
    return inst->may_read() != Option::NO;
}

bool Node::may_write() const {
    return inst->may_write() != Option::NO;
}

bool Node::may_access() const {
    return may_read() || may_write();
}

std::pair<const llvm::Value *, Address> Node::get_memory_address_pair() const {
    const auto *V = dynamic_cast<const MemoryInst&>(*inst).get_memory_operand();
    return *addr_refs.find(V);
}

bool Node::is_special() const {
    return inst->is_special();
}

Address Node::get_memory_address() const {
    return get_memory_address_pair().second;
}

z3::expr Node::exec(ExecMode mode) const {
    switch (mode) {
        case ExecMode::ARCH:  return arch;
        case ExecMode::TRANS: return trans;
        case ExecMode::EXEC:  return exec();
        default: std::abort();
    }
}

const char *to_string(ExecMode mode) {
    switch (mode) {
        case ExecMode::ARCH:  return "ARCH";
        case ExecMode::TRANS: return "TRANS";
        case ExecMode::EXEC:  return "EXEC";
        default: return "(invalid)";
    }
}

template <>
ExecMode from_string<ExecMode>(const std::string_view& s_) {
    std::string s {s_};
    std::transform(s.begin(), s.end(), s.begin(), static_cast<int (*)(int)>(&std::toupper));
    if (s == "ARCH") {
        return ExecMode::ARCH;
    } else if (s == "TRANS") {
        return ExecMode::TRANS;
    } else if (s == "EXEC") {
        return ExecMode::EXEC;
    } else {
        throw std::invalid_argument(util::to_string("bad ExecMode string '", s_, "'"));
    }
}

void Node::simplify() {
    arch = arch.simplify();
    trans = trans.simplify();
    constraints.simplify();
}


z3::expr Node::same_xstate(const Node& a, const Node& b) {
    z3::context& ctx = a.arch.ctx();
    if (a.is_special() || b.is_special()) {
        return ctx.bool_val(true);
    }
    if (a.xsaccess().is_false() || b.xsaccess().is_false()) {
        return ctx.bool_val(false);
    }
    return *a.xstate == *b.xstate;
}


z3::expr Node::xsaccess_order_less::operator()(NodeRef a, NodeRef b) const {
    if (a == b) { return aeg.context.FALSE; }
   const Node& an = aeg.lookup(a);
   const Node& bn = aeg.lookup(b);
    
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

bool Node::access_order_less::operator()(NodeRef a, NodeRef b) const {
    // TODO: imporve runtime
    const auto& order = aeg.po.reverse_postorder();
    const auto a_it = std::find(order.begin(), order.end(), a);
    const auto b_it = std::find(order.begin(), order.end(), b);
    return a_it < b_it;
}

z3::expr Node::get_member(const NodeRefSet &window, NodeRef ref, const z3::expr& val, const z3::expr &dfl) const {
    return window.contains(ref) ? val : dfl;
}

z3::expr Node::get_bool(const NodeRefSet &window, NodeRef ref, const z3::expr& val, bool dfl) const {
    return get_member(window, ref, val, ctx().bool_val(dfl));
}

z3::expr Node::get_arch(const NodeRefSet& window, NodeRef ref) const {
    return get_bool(window, ref, arch, false);
}

z3::expr Node::get_trans(const NodeRefSet& window, NodeRef ref) const {
    return get_bool(window, ref, trans, false);
}

z3::expr Node::get_read(const NodeRefSet& window, NodeRef ref) const {
    return get_bool(window, ref, read, false);
}

z3::expr Node::get_write(const NodeRefSet& window, NodeRef ref) const {
    return get_bool(window, ref, write, false);
}

z3::expr Node::get_xsread(const NodeRefSet& window, NodeRef ref) const {
    return get_bool(window, ref, xsread, false);
}

z3::expr Node::get_xswrite(const NodeRefSet& window, NodeRef ref) const {
    return get_bool(window, ref, xswrite, false);
}


}
