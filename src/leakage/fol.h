#pragma once

#include <memory>
#include <variant>
#include <optional>
#include <type_traits>
#include <vector>

#include "aeg/node.h"
#include "aeg/edge.h"

namespace lkg::fol {

struct Expr {
    using Ptr = std::unique_ptr<Expr>;
    enum ExprKind { EDGE, NODE, OP };
    virtual ExprKind expr_kind() const noexcept = 0;
    
    static Ptr parse(const std::string& s);
    
    virtual ~Expr() {}
    
protected:
    template <class T>
    static Ptr make_ptr(T&& x) {
        return std::make_unique(x);
    }
};

class Parser {
public:
    Expr::Ptr parse(const std::string& s) const;
    
private:
    using Args = std::vector<std::string_view>;
    Expr::Ptr parse_expr(std::string_view s) const;
    Expr::Ptr parse_node(std::string_view s) const;
    Expr::Ptr parse_edge(std::string_view s) const;
    Expr::Ptr parse_op(std::string_view s) const;
    Expr::Ptr parse_unop(const Args& args) const;
    Expr::Ptr parse_binop(const Args& args) const;
    Args arguments(std::string_view s) const;
};

struct Edge: Expr {
    using Kind = aeg::Edge::Kind;
    Kind kind;
    
    Edge(Kind kind): kind(kind) {}
    
    virtual ExprKind expr_kind() const noexcept override { return EDGE; }
};

struct Node: Expr {
    using Kind = aeg::ExecMode;
    Kind kind;
    
    Node(Kind kind): kind(kind) {}
    
    virtual ExprKind expr_kind() const noexcept override { return NODE; }
};

struct Op: Expr {
    enum OpKind {
        STAR,
        UNION,
        JOIN,
    };
    
    virtual ExprKind expr_kind() const noexcept override { return OP; }
    virtual OpKind op_kind() const noexcept = 0;
};

struct UnOp: Op {
    Ptr a;
    
    UnOp(Ptr&& a): a(std::move(a)) {}
};

struct StarOp: UnOp {
    std::optional<unsigned> min;
    std::optional<unsigned> max;
    
    StarOp(Ptr&& a): UnOp(std::move(a)) {}
    
    virtual OpKind op_kind() const noexcept override { return STAR; }
};

struct BinOp: Op {
    Ptr a;
    Ptr b;
    
    BinOp(Ptr&& a, Ptr&& b): a(std::move(a)), b(std::move(b)) {}
};

struct UnionOp: BinOp {
    UnionOp(Ptr&& a, Ptr&& b): BinOp(std::move(a), std::move(b)) {}
    
    virtual OpKind op_kind() const noexcept override { return UNION; }
};

struct JoinOp: BinOp {
    JoinOp(Ptr&& a, Ptr&& b): BinOp(std::move(a), std::move(b)) {}
    
    virtual OpKind op_kind() const noexcept override { return JOIN; }
};





/** Generate DAG from an expr. */
class Generator {
public:
    Generator(aeg::AEG& aeg): aeg(aeg) {}
    
    void generate(const Expr& e);
    
private:
    aeg::AEG& aeg;
    
    void generate_expr(const Expr& expr);
    void generate_node(const Node& node);
    void generate_edge(const Edge& edge);
    void generate_op(const Op& op);
};

}
