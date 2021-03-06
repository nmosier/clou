#pragma once

#include <memory>
#include <optional>
#include <unordered_map>

#include <z3++.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>

#include "aeg/fwd.h"
#include "aeg/address.h"
#include "aeg/constraints.h"
#include "noderef.h"
#include "aeg/taint.h"


struct Inst;

namespace aeg {

enum XSAccessType {
    XSREAD, XSWRITE
};

enum Access {
    READ, WRITE
};

template <class T>
T from_string(const std::string_view& s) {
    std::abort();
}

enum ExecMode: unsigned {ARCH, TRANS, EXEC};
const char *to_string(ExecMode mode);

inline std::ostream& operator<<(std::ostream& os, ExecMode mode) {
    return os << to_string(mode);
}

inline llvm::raw_ostream& operator<<(llvm::raw_ostream& os, ExecMode mode) {
    return os << to_string(mode);
}

template <> ExecMode from_string<ExecMode>(const std::string_view& s);

struct Node {
    std::unique_ptr<Inst> inst;
    z3::expr arch;  // bool: program order variable
    z3::expr trans; // bool: transient fetch order variable
    std::optional<Address> addr_def;
    std::optional<z3::expr> xstate;
    std::unordered_map<const llvm::Value *, Address> addr_refs;
    z3::expr read;
    z3::expr write;
    z3::expr xsread;
    z3::expr xswrite;
    std::optional<z3::expr> xsaccess_order; // int (atomic xread and/or xswrite)
    Taint attacker_taint;
    // Taint constant_taint;
    
    Constraints constraints;
    
    int stores_in;
    int stores_out;
    
    z3::context& ctx() const { return arch.ctx(); }
    
    z3::expr exec() const { return arch || trans; }
    z3::expr exec(ExecMode mode) const;
    z3::expr xsaccess() const { return xsread || xswrite; }
    z3::expr access() const { return read || write; }
    
    z3::expr get_arch(const NodeRefSet& window, NodeRef ref) const;
    z3::expr get_trans(const NodeRefSet& window, NodeRef ref) const;
    z3::expr get_read(const NodeRefSet& window, NodeRef ref) const;
    z3::expr get_write(const NodeRefSet& window, NodeRef ref) const;
    z3::expr get_xsread(const NodeRefSet& window, NodeRef ref) const;
    z3::expr get_xswrite(const NodeRefSet& window, NodeRef ref) const;

private:
    z3::expr get_member(const NodeRefSet& window, NodeRef ref, const z3::expr& val, const z3::expr& dfl) const;
    z3::expr get_bool(const NodeRefSet& window, NodeRef ref, const z3::expr& val, bool dfl) const;
    
public:
    
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

    void simplify();
    
    static z3::expr same_xstate(const Node& a, const Node& b);
    
    z3::expr same_xstate(const Node& other) const {
        return same_xstate(*this, other);
    }
    
    Node(std::unique_ptr<Inst>&& inst, Context& c);
    
    bool may_read() const;
    bool may_write() const;
    bool may_access() const;
    
    std::pair<const llvm::Value *, Address> get_memory_address_pair() const;
    
    Address get_memory_address() const;
    
    bool is_special() const;
    
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



}
