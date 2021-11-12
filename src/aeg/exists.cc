#include "aeg.h"
#include "cfg/expanded.h"
#include "util/z3.h"

namespace aeg {

z3::expr AEG::exists(Edge::Kind kind, NodeRef src, NodeRef dst) {
    switch (kind) {
        case Edge::CO: return co_exists(src, dst);
        case Edge::RF: return rf_exists(src, dst);
        case Edge::FR: return fr_exists(src, dst);
        case Edge::COX: return cox_exists(src, dst);
        case Edge::RFX: return rfx_exists(src, dst);
        case Edge::FRX: return frx_exists(src, dst);
            // case Edge::TFO: return tfo_exists(src, dst);
        case Edge::ADDR:
        case Edge::CTRL: {
            if (const Edge *edge = find_edge(src, dst, kind)) {
                return edge->exists;
            } else {
                return context.FALSE;
            }
        }
            
        default: std::abort();
    }
}

z3::expr AEG::exists_src(Edge::Kind kind, NodeRef src) const {
    const Node& node = lookup(src);
    switch (kind) {
        case Edge::PO: return node.arch;
        case Edge::TFO: return node.exec();
        case Edge::RF: return node.arch && node.write;
        case Edge::CO: return node.arch && node.write;
        case Edge::FR: return node.arch && node.read;
        case Edge::RFX: return node.exec() && node.xswrite;
        case Edge::COX: return node.exec() && node.xswrite;
        case Edge::FRX: return node.exec() && node.xsread;
        case Edge::ADDR: return node.exec() && node.read;
        case Edge::CTRL: return node.exec() && node.read;
        default: std::abort();
    }
}

z3::expr AEG::exists_dst(Edge::Kind kind, NodeRef dst) const {
    const Node& node = lookup(dst);
    switch (kind) {
        case Edge::PO: return node.arch;
        case Edge::TFO: return node.exec();
        case Edge::RF: return node.arch && node.read;
        case Edge::CO: return node.arch && node.write;
        case Edge::FR: return node.arch && node.write;
        case Edge::RFX: return node.exec() && node.xsread;
        case Edge::COX: return node.exec() && node.xswrite;
        case Edge::FRX: return node.exec() && node.xswrite;
        case Edge::ADDR: return node.exec() && node.access();
        case Edge::CTRL: return node.exec() && node.access();
        default: std::abort();
    }
}


/* COM */

z3::expr AEG::com_exists_precond(NodeRef src, NodeRef dst, Access src_kind, Access dst_kind) const {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    const auto check_kind = [&] (const Node& node, Access kind) -> bool {
        switch (kind) {
            case READ: return node.may_read();
            case WRITE: return node.may_write();
            default: std::abort();
        }
    };
    
    if (!check_kind(src_node, src_kind) || !check_kind(dst_node, dst_kind)) {
        return context.FALSE;
    }
    
    Node::access_order_less less {*this};
    if (!less(src, dst)) {
        return context.FALSE;
    }
    
    return src_node.arch && dst_node.arch && src_node.same_addr(dst_node);
}

z3::expr AEG::rf_exists(NodeRef src, NodeRef dst) {
    const z3::expr precond = com_exists_precond(src, dst, WRITE, READ);
    if (precond.is_false()) { return context.FALSE; }
    
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    if (src_node.is_special() && dst_node.is_special()) {
        return context.bool_val(src_node.inst->is_entry() && dst_node.inst->is_exit()) && src_node.arch && dst_node.arch;
    }
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    const auto get_val = [&] (NodeRef ref) -> z3::expr {
        return context.context.bool_val(src == ref);
    };
    
    const z3::expr seed = get_val(entry);
    z3::expr mem = z3::const_array(context.context.int_sort(), seed);
    const z3::expr addr = src == entry ? dst_node.get_memory_address() : src_node.get_memory_address();
    for (NodeRef ref : order) {
        if (ref == dst) { break; }
        const Node& node = lookup(ref);
        if (node.may_write() && node.inst->is_memory()) {
            mem = z3::conditional_store(mem, node.get_memory_address(), get_val(ref), node.arch && node.write);
        }
    }

    return precond && mem[addr];
}

z3::expr AEG::co_exists(NodeRef src, NodeRef dst) {
    return com_exists_precond(src, dst, WRITE, WRITE);
}

z3::expr AEG::fr_exists(NodeRef src, NodeRef dst) {
    return com_exists_precond(src, dst, READ, WRITE);
}


/* COMX */

z3::expr AEG::comx_exists_precond(NodeRef src, NodeRef dst, XSAccessType src_kind, XSAccessType dst_kind) const {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    const z3::expr src_access = src_node.xsaccess(src_kind);
    const z3::expr dst_access = dst_node.xsaccess(dst_kind);
    
    // TODO: make xsaccess return optional.
    if (src_access.simplify().is_false() || dst_access.simplify().is_false()) {
        return context.FALSE;
    }
    return src_access && dst_access && src_node.exec() && dst_node.exec() && src_node.same_xstate(dst_node);
}

z3::expr AEG::rfx_exists(NodeRef src, NodeRef dst) const {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    
    if (src_node.inst->is_entry() && dst_node.inst->is_exit()) {
        return src_node.exec() && dst_node.exec();
    }
    
    const Node::xsaccess_order_less less {*this};
    const z3::expr precond = comx_exists_precond(src, dst, XSWRITE, XSREAD);
    if (precond.is_false()) { return context.FALSE; }

    const z3::expr cond = less(src, dst);
    z3::expr intervening = context.FALSE;
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (node.xswrite.is_false()) { continue; }
        
        // TODO: this can be simplified in most cases -- only need 1 xstate comparison unless dealing with special nodes.
        intervening = intervening || node.exec() && node.xswrite && node.same_xstate(src_node) && node.same_xstate(dst_node) && less(src, ref) && less(ref, dst);
    }
    
    return precond && cond && !intervening;
}

z3::expr AEG::cox_exists(NodeRef src, NodeRef dst) const {
    const z3::expr precond = comx_exists_precond(src, dst, XSWRITE, XSWRITE);
    if (precond.is_false()) { return context.FALSE; }
    const z3::expr cond = Node::xsaccess_order_less(*this)(src, dst);
    return precond && cond;
}

z3::expr AEG::frx_exists(NodeRef src, NodeRef dst) const {
    const z3::expr precond = comx_exists_precond(src, dst, XSREAD, XSWRITE);
    if (precond.is_false()) { return context.FALSE; }
    const Node::xsaccess_order_less less {*this};
    return precond && less(src, dst);
}


}
