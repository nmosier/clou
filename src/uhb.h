#pragma once

#include <unordered_map>
#include <string>
#include <utility>
#include <vector>

#include <z3++.h>

#include "util.h"
#include "inst.h"
#include "config.h"

enum XSAccess {
    XSREAD, XSWRITE
};

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

class UHBContext {
public:
    UHBContext();
    
    z3::context context;
    const z3::expr TRUE;
    const z3::expr FALSE;
    
    z3::expr make_bool(const std::string& s = "") {
        return context.bool_const(get_name(s).c_str());
    }
    
    z3::expr make_int(const std::string& s = "") {
        return context.int_const(get_name(s).c_str());
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

struct UHBConstraints {
    std::vector<std::pair<z3::expr, std::string>> exprs;
    
    UHBConstraints() {}
    explicit UHBConstraints(const z3::expr& expr, const std::string& name): exprs({{expr, name}}) {}
    
    void add_to(z3::solver& solver) const;
    
    void operator()(const z3::expr& clause, const std::string& name);
    
    void simplify();
    
    z3::expr get(z3::context& ctx) const {
        return std::transform_reduce(exprs.begin(), exprs.end(), ctx.bool_val(true), util::logical_and<z3::expr>(), [] (const auto& pair) -> z3::expr {
            return pair.first;
        });
    }
};

std::ostream& operator<<(std::ostream& os, const UHBConstraints& c);

struct UHBAddress {
    z3::expr arch;
    z3::expr trans;
    UHBAddress(UHBContext& ctx): arch(ctx.make_int("arch")), trans(ctx.make_int("trans")) {}
    UHBAddress(const z3::expr& arch, const z3::expr& trans): arch(arch), trans(trans) {}
};

inline std::ostream& operator<<(std::ostream& os, const UHBAddress& x) {
    return os << "(arch)" << x.arch << " (trans)" << x.trans;
}

struct UHBNode {
    Inst inst;
    z3::expr arch;  // bool: program order variable
    z3::expr trans; // bool: transient fetch order variable
    z3::expr trans_depth; // int: transient depth
    z3::expr introduces_trans; // bool: introduces transient execution
    std::optional<UHBAddress> addr_def;
    z3::expr xstate;
    std::unordered_map<const llvm::Value *, UHBAddress> addr_refs;
    z3::expr xsread;
    z3::expr xswrite;
    z3::expr arch_order; // int
    z3::expr exec_order; // int
    z3::expr trans_group_min; // int
    z3::expr trans_group_max; // int
    z3::expr xsread_order; // int
    z3::expr xswrite_order; // int
    z3::expr mem; // int -> int
    z3::expr taint; // bool
    z3::expr taint_mem; // int -> bool
    UHBConstraints constraints;
    
    z3::expr exec() const { return arch || trans; }
    
    z3::expr get_addr_def() const {
        // TODO: Do we need to add an extra ite to qualify based on tfo bool too?
        return z3::ite(arch, addr_def->arch, addr_def->trans);
    }
    
    std::pair<const llvm::Value *, UHBAddress> get_addr_ref_pair(std::size_t idx) const {
        const llvm::Value *V = inst.I->getOperand(idx);
        return *addr_refs.find(V);
    }
    
    template <typename Derived>
    bool isa() const {
        return inst.isa<Derived>();
    }
    
    void simplify();
    
    static z3::expr same_addr(const UHBNode& a, const UHBNode& b);
    
    z3::expr same_addr(const UHBNode& other) const {
        return same_addr(*this, other);
    }

    static z3::expr same_xstate(const UHBNode& a, const UHBNode& b);
    
    z3::expr same_xstate(const UHBNode& other) const {
        return same_xstate(*this, other);
    }
    
    z3::expr get_xsaccess(XSAccess kind) const;
    const z3::expr& get_xsaccess_order(XSAccess kind) const {
        switch (kind) {
            case XSREAD: return xsread_order;
            case XSWRITE: return xswrite_order;
        }
    }
    
    UHBNode(const Inst& inst, UHBContext& c);
    
    bool is_write() const {
        return inst.kind == Inst::WRITE || inst.kind == inst.ENTRY;
    }
    
    bool is_read() const {
        return inst.kind == Inst::READ || inst.kind == inst.EXIT;
    }
    
    bool is_memory_op() const {
        return is_write() || is_read();
    }
    
    std::pair<const llvm::Value *, UHBAddress> get_memory_address_pair() const {
        assert(is_memory_op());
        switch (inst.kind) {
            case Inst::READ:
                return get_addr_ref_pair(0);
            case Inst::WRITE:
                return get_addr_ref_pair(1);
            default:
                std::abort();
        }
    }
    
    z3::expr get_memory_address() const {
        const UHBAddress& addr = get_memory_address_pair().second;
        return z3::ite(arch, addr.arch, addr.trans);
    }
    
    bool is_special() const;
};

struct UHBEdge {
#define UHBEDGE_KIND_X(X)                       \
X(PO)                                        \
X(TFO)                                       \
X(RF)                                        \
X(CO)                                        \
X(FR)                                        \
X(RFX)                                       \
X(COX)                                       \
X(FRX) \
X(ADDR)
    
#define UHBEDGE_KIND_E(name) name,
    enum Kind {
        UHBEDGE_KIND_X(UHBEDGE_KIND_E)
    };
#undef UHBEDGE_KIND_E
    
    Kind kind;
    z3::expr exists;
    UHBConstraints constraints;
    
    static const char *kind_tostr(Kind kind);
    const char *kind_tostr() const { return kind_tostr(kind); }
    static Kind kind_fromstr(const std::string& s);
    
    UHBEdge(Kind kind, UHBContext& ctx, const std::string& name = ""):
    kind(kind), exists(ctx.make_bool(name)) {}
    
    UHBEdge(Kind kind, const z3::expr& exists): kind(kind), exists(exists) {}
    
    struct Hash {
        size_t operator()(const UHBEdge& x) const { return std::hash<Kind>()(x.kind); }
    };
    
    bool operator==(const UHBEdge& other) const { return kind == other.kind; }
    void simplify() { constraints.simplify(); }
    
    bool possible() const {
        auto e = exists && constraints.get(exists.ctx());
        if constexpr (simplify_before_checking_for_impossible_edges) {
            e = e.simplify();
        }
        return !e.is_false();
    }
};

std::ostream& operator<<(std::ostream& os, const UHBEdge& e);
inline std::ostream& operator<<(std::ostream& os, UHBEdge::Kind kind) {
    return os << UHBEdge::kind_tostr(kind);
}
