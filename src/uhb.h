#pragma once

#include <unordered_map>
#include <string>
#include <utility>
#include <vector>

#include <z3++.h>

#include "util.h"
#include "inst.h"

enum XSAccess {
    XSREAD, XSWRITE
};

class UHBContext {
public:
    UHBContext();
    
    z3::context context;
    
    z3::expr make_bool(const std::string& s = "") {
        return context.bool_const(get_name(s).c_str());
    }
    
    z3::expr make_int(const std::string& s = "") {
        return context.int_const(get_name(s).c_str());
    }
    
    const z3::expr TRUE;
    const z3::expr FALSE;
    
    z3::expr to_expr(bool b) const {
        return b ? TRUE : FALSE;
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
    
    void operator()(const z3::expr& clause, const std::string& name = "");
    
    void simplify();
};

std::ostream& operator<<(std::ostream& os, const UHBConstraints& c);

struct UHBAddress {
    z3::expr arch;
    z3::expr trans;
    UHBAddress(UHBContext& ctx): arch(ctx.make_int("arch")), trans(ctx.make_int("trans")) {}
};

inline std::ostream& operator<<(std::ostream& os, const UHBAddress& x) {
    return os << "(arch)" << x.arch << " (trans)" << x.trans;
}

struct UHBNode {
    Inst inst;
    z3::expr arch;  // program order variable
    z3::expr trans; // transient fetch order variable
    z3::expr trans_depth; // transient depth
    std::optional<UHBAddress> addr_def;
    std::vector<std::pair<const llvm::Value *, UHBAddress>> addr_refs;
    z3::expr xsread;
    z3::expr xswrite;
    UHBConstraints constraints;
    
    z3::expr get_addr_def() const {
        // TODO: Do we need to add an extra ite to qualify based on tfo bool too?
        return z3::ite(arch, addr_def->arch, addr_def->trans);
    }
    
    z3::expr get_addr_ref(std::size_t idx) const {
        const UHBAddress& addr = addr_refs.at(idx).second;
        return z3::ite(arch, addr.arch, addr.trans);
    }
    
    z3::expr get_exec() const {
        return arch || trans;
    }
    
    void simplify();
    
    static z3::expr same_addr(const UHBNode& a, const UHBNode& b);
    
    z3::expr same_addr(const UHBNode& other) const {
        return same_addr(*this, other);
    }
    
    z3::expr same_xstate(const UHBNode& other) const {
        return same_addr(other);
    }
    
    z3::expr get_xsaccess(XSAccess kind) const;
    
    UHBNode(const Inst& inst, UHBContext& c);
};

struct UHBEdge {
#define UHBEDGE_KIND_X(X)                       \
X(FORK)                                      \
X(PO)                                        \
X(TFO)                                       \
X(RF)                                        \
X(CO)                                        \
X(FR)                                        \
X(RFX)                                       \
X(COX)                                       \
X(FRX)
    
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
};

std::ostream& operator<<(std::ostream& os, const UHBEdge& e);
inline std::ostream& operator<<(std::ostream& os, UHBEdge::Kind kind) {
    return os << UHBEdge::kind_tostr(kind);
}
