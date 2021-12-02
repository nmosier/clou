#include "leakage/fol.h"

namespace lkg::fol {

Expr::Ptr spectre_v1_classic() {
    auto addr1 = std::make_unique<Edge>(Edge(Edge::Kind::ADDR_GEP));
    auto addr2 = std::make_unique<Edge>(Edge(Edge::Kind::ADDR));
    auto data = std::make_unique<Edge>(Edge(Edge::Kind::DATA));
    auto rf = std::make_unique<Edge>(Edge(Edge::Kind::RF));
    auto data_rf = std::make_unique<JoinOp>(std::move(data), std::move(rf));
    auto tb = std::make_unique<UnionOp>(std::move(addr2), std::move(data_rf));
    auto tb_any = std::make_unique<StarOp>(std::move(tb));
    auto j1 = std::make_unique<JoinOp>(std::move(addr1), std::move(tb_any));
    auto addr3 = std::make_unique<Edge>(Edge(Edge::Kind::ADDR));
    auto ctrl = std::make_unique<Edge>(Edge(Edge::Kind::CTRL));
    auto addr_ctrl = std::make_unique<UnionOp>(std::move(addr3), std::move(ctrl));
    auto j2 = std::make_unique<JoinOp>(std::move(j1), std::move(addr_ctrl));
    auto trans = std::make_unique<Node>(Node::Kind::TRANS);
    auto j3 = std::make_unique<JoinOp>(std::move(j2), std::move(trans));
    return j3;
}


Expr::Ptr Parser::parse(const std::string& s) const {
    return parse_expr(s);
}

Expr::Ptr Parser::parse_expr(std::string_view s) const {
    if (s.empty()) {
        throw std::invalid_argument("parse_expr on empty string");
    }
    
    try {
        return parse_node(s);
    } catch (...) {}
    
    try {
        return parse_edge(s);
    } catch (...) {}
    
    try {
        return parse_op(s);
    } catch (...) {}
    
    throw std::invalid_argument("no valid expr parse for string");
}

Expr::Ptr Parser::parse_node(std::string_view s) const {
    try {
        const auto node_kind = aeg::from_string<aeg::ExecMode>(s);
        return std::make_unique<Node>(node_kind);
    } catch (...) {
        throw std::invalid_argument("parse_node on non-node");
    }
}

Expr::Ptr Parser::parse_edge(std::string_view s) const {
    try {
        const auto edge_kind = aeg::Edge::kind_fromstr(std::string(s));
        return std::make_unique<Edge>(edge_kind);
    } catch (...) {
        throw std::invalid_argument("parse_edge failed");
    }
}

Expr::Ptr Parser::parse_op(std::string_view s) const {
    if (s.front() == '(' && s.back() == ')') {
        s.remove_prefix(1);
        s.remove_suffix(1);
        const auto args = arguments(s);
        assert(!args.empty());
        
        try {
            return parse_unop(args);
        } catch (...) {}
        
        try {
            return parse_binop(args);
        } catch (...) {}
        
        throw std::invalid_argument("parse_op failed: no subparse");
        
    } else {
        throw std::invalid_argument("parse_op failed: missing parentheses");
    }
}

Expr::Ptr Parser::parse_unop(const Args& args) const {
    std::abort(); // TOdO
}

Expr::Ptr Parser::parse_binop(const Args& args) const {
    std::abort(); // TODO
}


Parser::Args Parser::arguments(std::string_view s) const {
    Args tokens;
    
    while (!s.empty()) {
        /* find first space with balanced parentheses */
        unsigned parens = 0;
        auto it = s.begin();
        for (; it != s.end(); ++it, ++parens) {
            switch (*it) {
                case '(':
                    ++parens;
                    break;
                    
                case ')':
                    if (parens == 0) {
                        throw std::invalid_argument("unbalanced parentheses");
                    }
                    --parens;
                    break;
                    
                case ' ':
                    if (parens == 0) {
                        goto found;
                    }
                    break;
                    
                default:
                    break;
            }
        }
        
        if (parens != 0) {
            throw std::invalid_argument("unbalanced parentheses");
        }
        
    found:
        
        const auto size = it - s.begin();
        tokens.push_back(s.substr(0, size));
        s.remove_prefix(size + 1);
    }

    return tokens;
}

}
