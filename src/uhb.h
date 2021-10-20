#pragma once

#include <unordered_map>
#include <string>
#include <utility>
#include <vector>
#include <variant>
#include <numeric>

#include <z3++.h>

#include "inst.h"
#include "config.h"
#include "noderef.h"
#include "util/functional.h"

namespace aeg {

class AEG;

enum Access {
    READ, WRITE
};

enum ExecMode {ARCH, TRANS, EXEC};

struct TupleSort {
    z3::context& ctx;
    z3::func_decl cons;
    z3::func_decl_vector projs;
    z3::sort sort;

    TupleSort(z3::context& ctx, const std::string& name, unsigned n, const char *names[], const z3::sort sorts[]): ctx(ctx), cons(ctx), projs(ctx), sort(ctx) {
        cons = ctx.tuple_sort(name.c_str(), n, names, sorts, projs);
        sort = cons.range();
    }
};

class Context {
public:
    Context();
    
    z3::context context;
    const z3::expr TRUE;
    const z3::expr FALSE;
    
    z3::expr make_bool(const std::string& s = "") {
        return context.bool_const(get_name(s).c_str());
    }
    
    z3::expr make_int(const std::string& s = "") {
        return context.int_const(get_name(s).c_str());
    }
    
    z3::expr make_const(const std::string& s, const z3::sort& sort) {
        return context.constant(get_name(s).c_str(), sort);
    }
        
    z3::expr bool_val(bool b) const {
        return b ? TRUE : FALSE;
    }
    
    operator z3::context& () {
        return context;
    }
    
private:
    std::unordered_map<std::string, unsigned> next;
    
    std::string get_name(const std::string& s) {
        unsigned& idx = next[s];
        return s + std::to_string(idx++);
    }
};

extern unsigned constraint_counter;

struct Constraints {
    std::vector<std::pair<z3::expr, std::string>> exprs;
    
    Constraints() {}
    explicit Constraints(const z3::expr& expr, const std::string& name): exprs({{expr, name}}) {}
    
    void add_to(z3::solver& solver) const;
    
    void operator()(const z3::expr& clause, const std::string& name);
    
    void simplify();
    
    z3::expr get(z3::context& ctx) const {
        return std::transform_reduce(exprs.begin(), exprs.end(), ctx.bool_val(true), util::logical_and<z3::expr>(), [] (const auto& pair) -> z3::expr {
            return pair.first;
        });
    }
};

std::ostream& operator<<(std::ostream& os, const Constraints& c);

// TODO: inline this
struct Address {
    z3::expr addr;
    
    operator const z3::expr& () const { return addr; }
    
    Address(Context& ctx): addr(ctx.make_int("addr")) {}
    Address(const z3::expr& addr): addr(addr) {}
    
    z3::expr operator==(const Address& other) const {
        return addr == other.addr;
    }
};

inline std::ostream& operator<<(std::ostream& os, const Address& x) {
    return os << "addr(" << x.addr;
}

struct XSAccess {
    enum Type {
        XSREAD, XSWRITE
    };
    
    struct All {};
    
    using State = std::variant<z3::expr, All>;
    using Order = std::variant<z3::expr, Entry, Exit>;
    
    z3::expr read;
    z3::expr write;
    State state;
    Order order;
    
    // XSAccess(const z3::expr& read, const z3::expr& write, const State& state, const Order& order): read(read), write(write), state(state), order(order) {}
};

enum XSAccessType {
    XSREAD, XSWRITE
};

struct Node {
    std::unique_ptr<Inst> inst;
    z3::expr arch;  // bool: program order variable
    z3::expr trans; // bool: transient fetch order variable
    z3::expr trans_depth; // int: transient depth
    z3::expr introduces_trans; // bool: introduces transient execution
    std::optional<Address> addr_def;
    std::optional<z3::expr> xstate;
    std::unordered_map<const llvm::Value *, Address> addr_refs;
    z3::expr read;
    z3::expr write;
    z3::expr xsread;
    z3::expr xswrite;
    z3::expr arch_order; // int
    z3::expr exec_order; // int
    z3::expr trans_group_min; // int
    z3::expr trans_group_max; // int
    std::optional<z3::expr> xsaccess_order; // int (atomic xread and/or xswrite)
    z3::expr mem; // int -> int
    z3::expr taint; // bool
    z3::expr taint_mem; // int -> bool
    z3::expr taint_trans;
    Constraints constraints;
    
    z3::context& ctx() const { return arch.ctx(); }

    z3::expr exec() const { return arch || trans; }
    z3::expr xsaccess() const { return xsread || xswrite; }
    z3::expr access() const { return read || write; }
    
    bool can_xsaccess() const { return can_xsread() || can_xswrite(); }
    
    const z3::expr& xsaccess(XSAccessType kind) const {
        switch (kind) {
            case XSREAD: return xsread;
            case XSWRITE: return xswrite;
            default: std::abort();
        }
    }
    
    bool can_xsread() const { return !xsread.is_false(); }
    bool can_xswrite() const { return !xswrite.is_false(); }

    // TODO: remove this.
    z3::expr get_addr_def() const { return *addr_def; }
    
    void simplify();
    
    static z3::expr same_addr(const Node& a, const Node& b);
    
    z3::expr same_addr(const Node& other) const {
        return same_addr(*this, other);
    }

    static z3::expr same_xstate(const Node& a, const Node& b);
    
    z3::expr same_xstate(const Node& other) const {
        return same_xstate(*this, other);
    }
    
    Node(std::unique_ptr<Inst>&& inst, Context& c);
    
    bool may_read() const { return inst->may_read() != Option::NO ; }
    bool may_write() const { return inst->may_write() != Option::NO; }
    bool may_access() const { return may_read() || may_write(); }
    
    std::pair<const llvm::Value *, Address> get_memory_address_pair() const {
        const auto *V = dynamic_cast<const MemoryInst&>(*inst).get_memory_operand();
        return *addr_refs.find(V);
    }
    
    z3::expr get_memory_address() const { return get_memory_address_pair().second; }
    
    bool is_special() const { return inst->is_special(); }
    
    struct xsaccess_order_less;
    struct access_order_less;
};

struct Node::xsaccess_order_less {
    const AEG& aeg;
    xsaccess_order_less(const AEG& aeg): aeg(aeg) {}
    z3::expr operator()(NodeRef a, NodeRef b) const;
};

struct Node::access_order_less {
    const AEG& aeg;
    access_order_less(const AEG& aeg): aeg(aeg) {}
    bool operator()(NodeRef a, NodeRef b) const;
};

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
    
    Edge(Kind kind, Context& ctx, const std::string& name = ""):
    kind(kind), exists(ctx.make_bool(name)) {}
    
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
