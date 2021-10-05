#include "aeg.h"
#include "progress.h"
#include "timer.h"
#include "util/z3.h"
#include "taint.h"
#include "taint_bv.h"

void AEG::construct(llvm::AliasAnalysis& AA, unsigned rob_size) {
    // initialize nodes
    std::transform(po.nodes.begin(), po.nodes.end(), std::back_inserter(nodes),
                   [&] (const auto& node) {
        const Inst inst =
        std::visit(util::creator<Inst>(),
                   node.v);
        return Node {inst, context};
    });
    
    // add entry, exit
    entry = 0;
    
    // TODO: This can be moved to CFG-Expanded, perhaps.
    for (NodeRef ref : node_range()) {
        if (lookup(ref).inst.kind == Inst::EXIT) {
            exits.insert(ref);
        }
    }
    
    for (NodeRef ref : node_range()) {
        graph.add_node(ref);
    }
    
    // print out some information
    const auto count_kind = [&] (Inst::Kind kind) {
        return std::count_if(nodes.begin(), nodes.end(), [kind] (const Node& node) -> bool {
            return node.inst.kind == kind;
        });
    };
    logv(2) << "Number of reads: " << count_kind(Inst::READ) << "\n";
    logv(2) << "Number of writes: " << count_kind(Inst::WRITE) << "\n";
    
    
    logv(2) << "Constructing nodes\n";
    construct_nodes();
    logv(2) << "Constructing po\n";
    construct_po();
    logv(2) << "Constructing tfo\n";
    construct_tfo2();
    logv(2) << "Constructing exec\n";
    construct_exec();
    logv(2) << "Constructing addr defs\n";
    construct_addr_defs();
    logv(2) << "Constructing addr refs\n";
    construct_addr_refs();
    logv(2) << "Constructing aliases\n";
    construct_aliases(AA);
    logv(2) << "Constructing arch order\n";
    construct_arch_order();
#if 0
    logv(2) << "Constructing exec order\n";
    construct_exec_order();
#endif
#if 0
    logv(2) << "Constructing trans group\n";
    construct_trans_group();
#endif
    logv(2) << "Constructing comx\n";
    construct_comx();
    logv(2) << "Constructing addr\n";
    construct_addr();
#if 0
    logv(2) << "Constructing mem\n";
    construct_mem();
#endif

#if 1
    construct_taint();
#endif
}

void AEG::construct_nodes() {
    // initialize `arch`
    // DEBUG: make sure that we hit all nodes
    // TODO: extract this to common function
    {
        std::vector<bool> done(size(), false);
        for (NodeRef ref : node_range()) {
            if (po.is_block_entry(ref)) {
                // beginning of basic block
                const z3::expr arch = ref == entry ? context.TRUE : context.make_bool("arch");
                NodeRef cur = ref;
                while (true) {
                    lookup(cur).arch = arch;
                    assert(!done[cur]);
                    done[cur] = true;
                    if (const auto succ = po.get_block_successor(cur)) {
                        cur = *succ;
                    } else {
                        break;
                    }
                }
            }
        }
        assert(std::all_of(done.begin(), done.end(), [] (bool b) { return b; }));
    }
    
    // initalize `trans`
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    {
        // TODO: rewrite as functoin?
        for (NodeRef ref : order) {
            const auto& preds = po.po.rev.at(ref);
            Node& node = lookup(ref);
            if (preds.size() != 1) {
                node.trans = context.FALSE;
            } else {
                const NodeRef pred = *preds.begin();
                const auto& pred_succs = po.po.fwd.at(pred);
                if (pred_succs.size() == 1 && lookup(pred).trans.is_false()) {
                    node.trans = context.FALSE;
                } else {
                    node.trans = context.make_bool("trans");
                }
            }
        }
    }
    
    // initialize `trans_depth`
    {
        for (NodeRef ref : node_range()) {
            Node& node = lookup(ref);
            if (node.trans.is_false()) {
                node.trans_depth = context.context.int_val(0);
            } else {
                node.trans_depth = context.make_int("trans_depth");
            }
        }
    }
    
    // initialize `xsread`, `xswrite`
    {
        for (NodeRef ref : node_range()) {
            Node& node = lookup(ref);
            node.xsread = node.xswrite = context.FALSE;
        }
    }
}

void AEG::construct_addr_defs() {
    for (Node& node : nodes) {
        if (node.inst.addr_def) {
            node.addr_def = UHBAddress {context};
        }
    }
}

void AEG::construct_addr_refs() {
    std::unordered_map<const llvm::Argument *, UHBAddress> main_args;
    std::unordered_map<const llvm::Constant *, UHBAddress> globals;
    
    for (NodeRef ref = 0; ref < size(); ++ref) {
        const AEGPO::Node& po_node = po.lookup(ref);
        Node& node = lookup(ref);
        
        for (const llvm::Value *V : node.inst.addr_refs) {
            const auto defs_it = po_node.refs.find(V);
            std::optional<UHBAddress> e;
            if (defs_it == po_node.refs.end()) {
                if (const llvm::ConstantData *CD = llvm::dyn_cast<llvm::ConstantData>(V)) {
                    if (CD->isNullValue()) {
                        const auto zero = context.context.int_val(0);
                        e = UHBAddress {context.context.int_val(0)};
                    } else {
                        llvm::errs() << "unhandled constant data: " << *CD << "\n";
                        std::abort();
                    }
                } else if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
                    auto main_args_it = main_args.find(A);
                    if (main_args_it == main_args.end()) {
                        main_args_it = main_args.emplace(A, UHBAddress {context}).first;
                    }
                    e = main_args_it->second;
                } else if (const llvm::Constant *G = llvm::dyn_cast<llvm::Constant>(V)) {
                    auto globals_it = globals.find(G);
                    if (globals_it == globals.end()) {
                        globals_it = globals.emplace(G, UHBAddress {context}).first;
                    }
                    e = globals_it->second;
                    llvm::errs() << "GLOBAL: " << *G << "\n" << *node.inst.I << "\n";
                } else {
                    auto& os = llvm::errs();
                    os << "Expected argument but got " << *V << "\n";
                    std::abort();
                }
            } else {
                const NodeRefSet& defs = defs_it->second;
                
                /* If defs only has one element (likely case), then we can just lookup that element's
                 * address definition integer. Otherwise, we define a new symbolic int that must be equal
                 * to one of the possiblities.
                 */
                const auto lookup_def = [&] (NodeRef def) {
                    return *lookup(def).addr_def;
                };
                if (defs.size() == 1) {
                    e = lookup_def(*defs.begin());
                } else {
                    e = UHBAddress {context};
                    if (defs.size() != 0) {
                        node.constraints(util::any_of<z3::expr>(defs.begin(), defs.end(),
                                                                [&] (NodeRef def) {
                            return lookup_def(def) == *e;
                        }, context.FALSE), "addr-ref");
                    }
                }
            }
            node.addr_refs.emplace(V, *e);
        }
    }
}

void AEG::construct_exec() {
    // TODO: test making this its own variable
    // NOTE: depends on results of construct_tfo().
    
    // exclusive architectural/transient execution
    for (const NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        std::stringstream ss;
        ss << "excl-exec-" << ref;
        node.constraints(!(node.arch && node.trans), ss.str());
    }
    
    construct_arch();
    construct_trans();
}
 
void AEG::construct_arch() {
    // Entry node is architecturally executed
    Node& entry_node = lookup(entry);
    entry_node.constraints(entry_node.arch, "entry-arch");
    
    constraints(std::transform_reduce(exits.begin(), exits.end(), context.FALSE, util::logical_or<z3::expr>(), [&] (NodeRef ref) -> z3::expr {
        return lookup(ref).arch;
    }), "exit-arch");
    // TODO: replace with "true", then substitute changes
}

void AEG::construct_trans() {
    // NOTE: depends on results of construct_tfo()
    
    // limit transient depth to speculation depth
    for (Node& node : nodes) {
        const auto f = z3::implies(node.trans, node.trans_depth <= context.context.int_val(num_specs()));
        node.constraints(f, "depth-limit-trans");
    }
    
    // arch resets transient depth
    for (Node& node : nodes) {
        const auto f = z3::implies(node.arch, node.trans_depth == context.context.int_val(0));
        node.constraints(f, "depth-reset");
    }
    
    // tfo increments transient depth
    // TODO: Perform substitution when possible
    for (const auto ref : node_range()) {
        const auto& preds = po.po.rev.at(ref);
        if (preds.size() == 1) {
            Node& node = lookup(ref);
            const NodeRef pred = *preds.begin();
            const auto f = z3::implies(node.trans, node.trans_depth == lookup(pred).trans_depth + context.context.int_val(1));
            node.constraints(f, "depth-add");
        }
    }
    
    // transient execution of node requires incoming tfo edge
    for (const auto ref : node_range()) {
        Node& node = lookup(ref);
        const auto tfos = get_edges(Direction::IN, ref, Edge::TFO);
        // TODO: experiment with using z3::{atleast,exactly}(vec, 1) instead.
        const z3::expr f = util::any_of(tfos.begin(), tfos.end(), [] (const auto& edge) -> z3::expr {
            return edge->exists;
        }, context.FALSE);
        node.constraints(z3::implies(node.trans, f), "trans-tfo");
    }
} 

void AEG::construct_po() {
    logv(3) << __FUNCTION__ << ": adding edges\n";
    
    std::size_t nedges = 0;
    for (const NodeRef src : node_range()) {
        Node& src_node = lookup(src);
        
        // add successor po edges
        for (const NodeRef dst : po.po.fwd.at(src)) {
            std::stringstream ss;
            ss << "po-" << src << "-" << dst;
            const z3::expr exists = add_optional_edge(src, dst, Edge {Edge::PO, src_node.arch && lookup(dst).arch}, ss.str());
            ++nedges;
        }
    }
    
    // add 'exactly one successor' constraint
    for (const NodeRef src : node_range()) {
        if (exits.find(src) != exits.end()) { continue; }
        const auto edges = get_edges(Direction::OUT, src, Edge::PO);
        z3::expr_vector vec {context};
        for (const auto& e : edges) {
            vec.push_back(e->exists);
        }
        Node& src_node = lookup(src);
        src_node.constraints(z3::implies(src_node.arch, z3::exactly(vec, 1)), "po-succ");
    }
    
    // add 'exactly one predecessor' constraint
    for (const NodeRef dst : node_range()) {
        if (dst == entry) { continue; }
        const auto edges = get_edges(Direction::IN, dst, Edge::PO);
        z3::expr_vector vec {context};
        for (const auto& e : edges) {
            vec.push_back(e->exists);
        }
        Node& dst_node = lookup(dst);
        dst_node.constraints(z3::implies(dst_node.arch, z3::exactly(vec, 1)), "po-pred");
    }
}

void AEG::construct_tfo2() {
    std::size_t nedges = 0;
    for (const NodeRef src : node_range()) {
        Node& src_node = lookup(src);
        z3::expr_vector tfos {context};
        for (const NodeRef dst : po.po.fwd.at(src)) {
            // add optional edge
            const Node& dst_node = lookup(dst);
            const z3::expr exists = add_optional_edge(src, dst, Edge {Edge::TFO,
                (src_node.arch && dst_node.trans && can_introduce_trans(src)) ||
                (src_node.trans && dst_node.trans)
            }, "tfo");
            ++nedges;
            tfos.push_back(exists);
        }
        
        // add 'at most one tfo successor' constraint
        if (exits.find(src) == exits.end()) {
            src_node.constraints(z3::implies(src_node.exec(), z3::atmost2(tfos, 1)), "tfo-succ");
        }
    }
    std::cerr << "added " << nedges << " tfo edges\n";
}

void AEG::construct_aliases(llvm::AliasAnalysis& AA) {
    using ID = AEGPO::ID;
    struct Info {
        ID id;
        const llvm::Value *V;
        z3::expr e;
        std::optional<NodeRef> ref;
    };
    std::vector<Info> addrs;
    std::unordered_map<std::pair<ID, const llvm::Value *>, NodeRef> seen;
    for (NodeRef i = 0; i < size(); ++i) {
        const Node& node = lookup(i);
        if (node.addr_def) {
            const ID& id = *po.lookup(i).id;
            const llvm::Value *V = node.inst.I;
            addrs.push_back({.id = id, .V = V, .e = *node.addr_def, .ref = i});
            [[maybe_unused]] const auto res = seen.emplace(std::make_pair(id, V), i);
            
            // TODO: ignore collisions for now, since they're introduced during the CFG expansion step.
        }
    }
    
    // check for arguments
    for (NodeRef i = 0; i < size(); ++i) {
        const Node& node = lookup(i);
        const AEGPO::Node& po_node = po.lookup(i);
        for (const llvm::Value *V : node.inst.addr_refs) {
            if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
                const ID id {po_node.id->func, {}};
                if (seen.emplace(std::make_pair(id, V), i).second) {
                    const auto it = std::find_if(node.addr_refs.begin(), node.addr_refs.end(),
                                                 [&] (const auto& p) {
                        return p.first == V;
                    });
                    assert(it != node.addr_refs.end());
                    addrs.push_back({.id = id, .V = V, .e = it->second, .ref = std::nullopt});
                }
            }
        }
    }
    
    std::cerr << addrs.size() << " addrs\n";
    
    // add constraints
    unsigned nos, musts, mays;
    nos = musts = mays = 0;
    for (auto it1 = addrs.begin(); it1 != addrs.end(); ++it1) {
        ValueLoc vl1 {it1->id, it1->V};
        alias_rel.emplace(std::make_pair(vl1, vl1), llvm::MustAlias);
        for (auto it2 = std::next(it1); it2 != addrs.end(); ++it2) {
            if (po.alias_valid(it1->id, it2->id)) {
                const auto alias_res = AA.alias(it1->V, it2->V);
                ValueLoc vl2 {it2->id, it2->V};
                
                const auto is_arch = [&] (const Info& x) -> z3::expr {
                    if (x.ref) {
                        return lookup(*x.ref).arch;
                    } else {
                        return context.TRUE;
                    }
                };
                
                const z3::expr arch1 = is_arch(*it1);
                const z3::expr arch2 = is_arch(*it2);
                const z3::expr precond = arch1 && arch2; // TODO: try simplifying?
                                
                switch (alias_res) {
                    case llvm::NoAlias:
                        constraints(z3::implies(precond, it1->e != it2->e), "no-alias");
                        ++nos;
                        break;
                    case llvm::MayAlias:
                        ++mays;
                        break;
                    case llvm::MustAlias:
                        constraints(z3::implies(precond, it1->e == it2->e), "must-alias");
                        ++musts;
                        break;
                    default: std::abort();
                }
                add_alias_result(vl1, vl2, alias_res);
            }
        }
    }
    std::cerr << "NoAlias: " << nos << "\n"
    << "MustAlias: " << musts << "\n"
    << "MayAlias: " << mays << "\n";
}

void AEG::construct_comx() {
    /* Set xsread, xswrite */
    NodeRefSet xsaccesses;
    
    enum Option {
        YES, NO, MAYBE
    };
    
    const auto process = [&] (NodeRef i, Node& node, Option xsread, Option xswrite) {
        const auto make_xsaccess = [&] (Option xsaccess, const std::string& name) {
            switch (xsaccess) {
                case YES: return context.TRUE;
                case NO: return context.FALSE;
                case MAYBE: return context.make_bool(name);
            }
        };
        node.xsread = make_xsaccess(xsread, "xsread");
        node.xswrite = make_xsaccess(xswrite, "xswrite");

        if (!node.is_special()) {
            node.xstate = context.make_int("xstate");
            node.constraints(*node.xstate == node.get_memory_address(), "xstate-addr-eq");
            if (xsread != NO || xswrite != NO) {
                xsaccesses.insert(i);
            }
        }
    };
    
    for (NodeRef i = 0; i < size(); ++i) {
        Node& node = lookup(i);
        struct Info {
            Option xsread;
            Option xswrite;
        };
        static const std::unordered_map<Inst::Kind, Info> map = {
            {Inst::READ, {YES, MAYBE}},
            {Inst::WRITE, {YES, YES}},
            {Inst::ENTRY, {NO, YES}},
            {Inst::EXIT, {YES, NO}},
        };
        const auto it = map.find(node.inst.kind);
        if (it != map.end()) {
            const Info& info = it->second;
            process(i, node, info.xsread, info.xswrite);
        }
    }
    
    logv(3) << "constructing xsaccess order...\n";
    construct_xsaccess_order(xsaccesses);
}

// TODO: need to come up with definitive po edges. Then we can reference these directly.

void AEG::construct_arch_order() {
    // add variables
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        if (ref == entry) {
            node.arch_order = context.context.int_val(0);
        } else {
            node.arch_order = context.make_int("arch_order");
        }
    }
    
    // add constraints
    for (NodeRef ref : node_range()) {
        if (ref == entry) { continue; }
        const auto& preds = po.po.rev.at(ref);
        Node& node = lookup(ref);
        
        z3::expr lower = context.TRUE;
        z3::expr exact = context.FALSE;
        for (NodeRef pred : preds) {
            Node& pred_node = lookup(pred);
            lower = lower && z3::implies(pred_node.arch, node.arch_order > pred_node.arch_order);
            exact = exact || (pred_node.arch && node.arch_order == pred_node.arch_order + 1);
        }
        node.constraints(z3::implies(node.arch, lower), "arch-order-lower");
        node.constraints(z3::implies(node.arch, exact), "arch-order-exact");
    }
}

void AEG::construct_exec_order() {
    // add variable
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        if (ref == entry) {
            node.exec_order = context.context.int_val(0);
        } else {
            node.exec_order = context.make_int("exec_order");
        }
    }
    
    // add constraints
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        const auto preds = get_nodes(Direction::IN, ref, Edge::TFO);
        for (const auto& pred_pair : preds) {
            const NodeRef pred = pred_pair.first;
            const z3::expr& cond = pred_pair.second;
            const Node& pred_node = lookup(pred);
            
            const z3::expr f = z3::implies(node.exec() && pred_node.exec() && cond, node.exec_order > pred_node.exec_order);
            node.constraints(f, "exec-order");
        }
    }
}

void AEG::construct_xsaccess_order(const NodeRefSet& xsaccesses) {
    // add variables
    for (NodeRef ref : xsaccesses) {
        Node& node = lookup(ref);
        node.xsaccess_order = context.make_int("xsaccess_order");
    }
    
    // require that all exits have same sequence number (not absolutely necessary)
    for (auto it1 = exits.begin(), it2 = std::next(it1); it2 != exits.end(); ++it1, ++it2) {
        constraints(lookup(*it1).xsread == lookup(*it2).xsread, "xswrite-exits-eq");
    }
    

#if 1
    // DEBUG: make them all distinct
    z3::expr_vector vec {context.context};
    for (NodeRef ref : xsaccesses) {
        vec.push_back(*lookup(ref).xsaccess_order);
    }
    constraints(z3::distinct(vec), "xsaccess-order-distinct");
#endif
}

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

z3::expr AEG::cox_exists(NodeRef src, NodeRef dst) const {
    const z3::expr precond = comx_exists_precond(src, dst, XSWRITE, XSWRITE);
    if (precond.is_false()) { return context.FALSE; }
    const z3::expr cond = UHBNode::xsaccess_order_less(*this)(src, dst);
    return precond && cond;
}

z3::expr AEG::rfx_exists(NodeRef src, NodeRef dst) const {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    
    if (src_node.inst.kind == Inst::ENTRY && dst_node.inst.kind == Inst::EXIT) {
        return src_node.exec() && dst_node.exec();
    }
    
    const UHBNode::xsaccess_order_less less {*this};
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

z3::expr AEG::dbg_intervening_xswrite(NodeRef src, NodeRef dst) {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    
    if (src_node.inst.kind == Inst::ENTRY && dst_node.inst.kind == Inst::EXIT) {
        return src_node.exec() && dst_node.exec();
    }
    
    const UHBNode::xsaccess_order_less less {*this};
    const z3::expr precond = comx_exists_precond(src, dst, XSWRITE, XSREAD);
    if (precond.is_false()) { return context.FALSE; }

    const z3::expr cond = less(src, dst);
    z3::expr intervening = context.context.int_val(-1);
    for (NodeRef ref : node_range()) {
        const Node& node = lookup(ref);
        if (node.xswrite.is_false()) { continue; }
        intervening = z3::ite(node.exec() && node.xswrite && node.same_xstate(src_node) && less(src, ref) && less(ref, dst), context.context.int_val((int) ref), intervening);
    }
    
    return intervening;
}

z3::expr AEG::frx_exists(NodeRef src, NodeRef dst) const {
    const z3::expr precond = comx_exists_precond(src, dst, XSREAD, XSWRITE);
    if (precond.is_false()) { return context.FALSE; }
    const UHBNode::xsaccess_order_less less {*this};
    return precond && less(src, dst);
}

z3::expr AEG::com_exists_precond(NodeRef src, NodeRef dst, Access src_kind, Access dst_kind) const {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    const auto check_kind = [&] (const Node& node, Access kind) -> bool {
        switch (kind) {
            case READ: return node.is_read();
            case WRITE: return node.is_write();
            default: std::abort();
        }
    };
    
    if (!check_kind(src_node, src_kind) || !check_kind(dst_node, dst_kind)) {
        return context.FALSE;
    }
    
    UHBNode::access_order_less less {*this};
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
        return context.bool_val(src_node.inst.kind == Inst::ENTRY && dst_node.inst.kind == Inst::EXIT) && src_node.arch && dst_node.arch;
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
        if (node.inst.kind == Inst::WRITE) {
            mem = z3::conditional_store(mem, node.get_memory_address(), get_val(ref), node.arch);
        }
    }

    return precond && mem[addr];
}

z3::expr AEG::fr_exists(NodeRef src, NodeRef dst) {
    return com_exists_precond(src, dst, READ, WRITE);
}

z3::expr AEG::co_exists(NodeRef src, NodeRef dst) {
    return com_exists_precond(src, dst, WRITE, WRITE);
}


void AEG::construct_addr() {
    /* addr dependencies can be between a read and a subsequent read/write
     */
    
    for (NodeRef dst : node_range()) {
        const Node& dst_node = lookup(dst);
        const AEGPO::Node& dst_po_node = po.lookup(dst);
        if (dst_node.inst.kind == Inst::READ || dst_node.inst.kind == Inst::WRITE) {
            const llvm::Value *V = dst_node.get_memory_address_pair().first;
            const auto refs_it = dst_po_node.refs.find(V);
            if (refs_it != dst_po_node.refs.end()) {
                const NodeRefSet& srcs = refs_it->second;
                // find all load operands that any of these sources depend on
                NodeRefSet loads;
                
                struct Info {
                    NodeRef ref;
                    enum State {
                        UNSEEN,
                        SEEN,
                    } state;
                };
                
                std::vector<Info> todo;
                std::transform(srcs.begin(), srcs.end(), std::back_inserter(todo), [] (NodeRef src) -> Info {
                    return {src, Info::UNSEEN};
                });
                
                while (!todo.empty()) {
                    const Info in = todo.back();
                    todo.pop_back();
                    const Node& node = lookup(in.ref);
                    const AEGPO::Node& po_node = po.lookup(in.ref);
                    
                    if (node.inst.kind == Inst::READ) {
                        if (in.state == Info::SEEN) {
                            add_unidir_edge(in.ref, dst, Edge {Edge::ADDR, node.exec() && dst_node.exec()});
                            break;
                        }
                    } else {
                        switch (in.state) {
                            case Info::UNSEEN:
                                if (const auto *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(node.inst.I)) {
                                    for (const llvm::Value *V : GEP->indices()) {
                                        const auto refs_it = po_node.refs.find(V);
                                        if (refs_it != po_node.refs.end()) {
                                            std::transform(refs_it->second.begin(), refs_it->second.end(), std::back_inserter(todo), [&] (NodeRef ref) -> Info {
                                                return Info {ref, Info::SEEN};
                                            });
                                        }
                                    }
                                } else {
                                    for (const auto& ref_pair : po_node.refs) {
                                        std::transform(ref_pair.second.begin(), ref_pair.second.end(), std::back_inserter(todo), [] (NodeRef ref) -> Info {
                                            return {ref, Info::UNSEEN};
                                        });
                                    }
                                }
                                break;
                                
                            case Info::SEEN:
                                for (const auto& ref_pair : po_node.refs) {
                                    std::transform(ref_pair.second.begin(), ref_pair.second.end(), std::back_inserter(todo), [] (NodeRef ref) -> Info {
                                        return {ref, Info::SEEN};
                                    });
                                }
                                break;
                        }
                    }
                }
            }
            
        }
    }
}


#if 0
/** Construct architectural memory state for each node. Dependencies: AEG::construct_xsaccess_order(), AEG::construct_addr_refs().
 */
void AEG::construct_mem() {
    std::vector<NodeRef> order;
    po.reverse_postorder(std::back_inserter(order));
    
    const z3::expr invalid = context.context.int_val(-1);
    const z3::sort mem_sort = context.context.array_sort(context.context.int_sort(), context.context.int_sort());

    for (NodeRef ref : order) {
        Node& node = lookup(ref);
        z3::expr& mem = node.mem;
        
        if (node.inst.kind == Inst::ENTRY) {
            mem = z3::const_array(context.context.int_sort(), *node.xsaccess_order);
        } else {
            const auto& preds = po.po.rev.at(ref);
            if (preds.size() == 1) {
                const NodeRef pred = *preds.begin();
                mem = lookup(pred).mem;
            } else {
#if 0
                std::stringstream ss;
                ss << "mem" << ref;
                mem = context.context.constant(ss.str().c_str(), mem_sort);
                for (NodeRef pred : preds) {
                    const Node& pred_node = lookup(pred);
                    try {
                        node.constraints(z3::implies(pred_node.arch, node.mem == pred_node.mem), "mem-pred");
                    } catch (const z3::exception& e) {
                        std::cerr << e.what() << "\n";
                        std::abort();
                    }
                }
#elif 0
                auto pred_it = preds.begin();
                mem = lookup(*pred_it).mem;
                for (++pred_it; pred_it != preds.end(); ++pred_it) {
                    const auto& pred = lookup(*pred_it);
                    mem = z3::ite(pred.arch, <#const expr &t#>, <#const expr &e#>)
                }
#endif
            }
            
            if (!node.xswrite.is_false()) {
                const z3::expr addr = z3::ite(node.xswrite, node.get_memory_address(), invalid);
                mem = z3::store(mem, addr, node.xsaccess_order);
            }
        }
    }
}
#endif


void AEG::construct_taint() {
    logv(2) << "Constructing taint\n";
    tainter = std::make_unique<Taint_Array>(*this);
    tainter->run();
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    // add transient execution taint pass
    for (NodeRef ref : order) {
        Node& node = lookup(ref);
        if (const auto pred = can_trans(ref)) {
            const Node& pred_node = lookup(*pred);
            node.taint_trans = pred_node.taint_trans;
            if (const auto *I = pred_node.inst.I) {
                if (const auto *B = llvm::dyn_cast<llvm::BranchInst>(pred_node.inst.I)) {
                    if (B->isConditional()) {
                        const auto *C = B->getCondition();
                        const auto cond_taint = tainter->get_value(*pred, C);
                        node.taint_trans = node.taint_trans || cond_taint;
                    }
                }
            }
            node.taint_trans = node.taint_trans && node.trans;
        } else {
            node.taint_trans = context.FALSE;
        }
    }
}
