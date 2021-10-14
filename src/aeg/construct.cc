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
        std::unique_ptr<Inst> inst(std::visit([] (const auto& x) {
            return Inst::Create(x);
        }, node.v));
        return Node {std::move(inst), context};
    });
    
    // add entry, exit
    entry = 0;
    
    // TODO: This can be moved to CFG-Expanded, perhaps.
    for (NodeRef ref : node_range()) {
        if (lookup(ref).inst->is_exit()) {
            exits.insert(ref);
        }
    }
    
    for (NodeRef ref : node_range()) {
        graph.add_node(ref);
    }
    
    // print out some information
    const auto count_kind = [&] (Inst::Kind kind) {
        return std::count_if(nodes.begin(), nodes.end(), [kind] (const Node& node) -> bool {
            return node.inst->kind() == kind;
        });
    };
    logv(2) << "Number of loads: " << count_kind(Inst::Kind::LOAD) << "\n";
    logv(2) << "Number of stores: " << count_kind(Inst::Kind::STORE) << "\n";
    
    
    logv(2) << "Constructing nodes\n";
    construct_nodes();
    logv(2) << "Constructing po\n";
    construct_po();
    logv(2) << "Constructing tfo\n";
    construct_tfo();
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
    logv(2) << "Constructing com\n";
    construct_com();
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
    logv(2) << "Constructing dependencies\n";
    construct_dependencies();
    logv(2) << "Constructing dominators\n";
    construct_dominators();
    logv(2) << "Constructing postdominators\n";
    construct_postdominators();
    
    // syntactic memory dependencies
    logv(2) << "Constructing addr\n";
    construct_addr();
    logv(2) << "Constructing data\n";
    construct_data();
    logv(2) << "Constructing ctrl\n";
    construct_ctrl();
#if 0
    logv(2) << "Constructing mem\n";
    construct_mem();
#endif

#if USE_TAINT
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
        // TODO: rewrite as function?
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
        if (auto *RI = dynamic_cast<RegularInst *>(node.inst.get())) {
            // TODO: this is fragmented. Try to unify addr_defs
            if (RI->addr_def) {
                node.addr_def = UHBAddress {context};
            }
        }
    }
}

void AEG::construct_addr_refs() {
    std::unordered_map<const llvm::Argument *, UHBAddress> main_args;
    std::unordered_map<const llvm::Constant *, UHBAddress> globals;
    
    for (NodeRef ref = 0; ref < size(); ++ref) {
        const CFG::Node& po_node = po.lookup(ref);
        Node& node = lookup(ref);
        
        if (const RegularInst *inst = dynamic_cast<const RegularInst *>(node.inst.get())) {
            
            for (const llvm::Value *V : inst->addr_refs) {
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
                        llvm::errs() << "GLOBAL: " << *G << "\n" << inst->I << "\n";
                    } else {
                        auto& os = llvm::errs();
                        os << "Expected argument but got " << *V << "\n";
                        os << "when looking at instruction " << *inst->I << "\n";
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

void AEG::construct_tfo() {
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
    using ID = CFG::ID;
    struct Info {
        ID id;
        const llvm::Value *V;
        z3::expr e;
        std::optional<NodeRef> ref;
    };
    std::vector<Info> addrs;
    std::unordered_map<std::pair<ID, const llvm::Value *>, NodeRef> seen;
    for (NodeRef i : node_range()) {
        const Node& node = lookup(i);
        if (node.addr_def) {
            const ID& id = *po.lookup(i).id;
            const llvm::Value *V = dynamic_cast<const RegularInst&>(*node.inst).I;
            addrs.push_back({.id = id, .V = V, .e = *node.addr_def, .ref = i});
            [[maybe_unused]] const auto res = seen.emplace(std::make_pair(id, V), i);
            
            // TODO: ignore collisions for now, since they're introduced during the CFG expansion step.
        }
    }
    
    // check for arguments
    for (NodeRef i = 0; i < size(); ++i) {
        const Node& node = lookup(i);
        const CFG::Node& po_node = po.lookup(i);
        if (const auto *inst = dynamic_cast<const RegularInst *>(node.inst.get())) {
            for (const llvm::Value *V : inst->addr_refs) {
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
                const z3::expr precond = alias_mode.transient ? context.TRUE : (arch1 && arch2);
                                
                switch (alias_res) {
                    case llvm::NoAlias:
                        constraints(z3::implies(precond, it1->e != it2->e), "no-alias");
                        ++nos;
                        break;
                    case llvm::MayAlias:
                        if (alias_mode.lax) {
                            constraints(z3::implies(precond, it1->e != it2->e), "may-alias");
                        }
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
    
    const auto process = [&] (NodeRef i, Node& node, Option xsread, Option xswrite) {
        const auto make_xsaccess = [&] (Option xsaccess, const std::string& name) {
            switch (xsaccess) {
                case Option::MUST: return context.TRUE;
                case Option::NO: return context.FALSE;
                case Option::MAY: return context.make_bool(name);
            }
        };
        node.xsread = make_xsaccess(xsread, "xsread");
        node.xswrite = make_xsaccess(xswrite, "xswrite");
        
        if (!node.is_special()) {
            if (xsread != Option::NO || xswrite != Option::NO) {
                node.xstate = context.make_int("xstate");
                node.constraints(*node.xstate == node.get_memory_address(), "xstate-addr-eq");
                xsaccesses.insert(i);
            }
        }
    };
    
    for (NodeRef i : node_range()) {
        Node& node = lookup(i);
        process(i, node, node.inst->may_xsread(), node.inst->may_xswrite());
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


void AEG::construct_addr() {
    /* Address dependencies are from a load to a subsequent access.
     * The address of the access should be dependent on the result of the load.
     * This means that the address operand of the access instruction should be the load or list it as a dependency.
     */
    for (NodeRef access_ref : node_range()) {
        const Node& access_node = lookup(access_ref);
        if (!access_node.may_access()) { continue; }
        const MemoryInst *access_inst = dynamic_cast<const MemoryInst *>(access_node.inst.get());
        if (access_inst == nullptr) { continue; }
        const llvm::Value *V = access_inst->get_memory_operand();
        const auto& access_po_node = po.lookup(access_ref);
        const auto addr_refs_it = access_po_node.refs.find(V);
        if (addr_refs_it == access_po_node.refs.end()) { continue; }
        const auto& addr_refs = addr_refs_it->second;
        for (const NodeRef addr_ref : addr_refs) {
            NodeRefSet candidate_srcs = dependencies.at(addr_ref);
            candidate_srcs.insert(addr_ref);
            for (NodeRef candidate_src : candidate_srcs) {
                const Node& candidate_node = lookup(candidate_src);
                if (candidate_node.may_read()) {
                    add_unidir_edge(candidate_src, access_ref, Edge {
                        Edge::ADDR,
                        (access_node.exec() && access_node.access()) && (candidate_node.exec() && candidate_node.read)
                    });
                }
            }
        }
    }
}

void AEG::construct_dependencies() {
    /* Compute map of noderefs to set of noderefs it depends on.
     * FORWARD pass
     * A node depends on itself? No for now.
     */
    
    std::unordered_map<NodeRef, DependencyMap> ins, outs;
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    for (const NodeRef dst : order) {
        // collect inputs
        NodeRefMap& in = ins[dst];
        for (const NodeRef src : po.po.rev.at(dst)) {
            in += outs.at(src);
        }
        
        // transform to output
        NodeRefMap& out = outs[dst] = in;
        const auto& node = po.lookup(dst);
        NodeRefSet& out_set = out[dst];
        for (const auto& ref_pair : node.refs) {
            for (const NodeRef ref_ref : ref_pair.second) {
                out_set.insert(ref_ref);
                const auto& ref_set = out.at(ref_ref);
                out_set.insert(ref_set.begin(), ref_set.end());
            }
        }
    }
    
    DependencyMap res;
    for (const auto& out : outs) {
        res += out.second;
    }
    
    dependencies = res;
}

AEG::DominatorMap AEG::construct_dominators_shared(Direction dir) const {
    /* At each program point, store the set of instructions that MUST have been executed to reach this instruction. This means that the MEET operator is set intersection.
     */
    std::unordered_map<NodeRef, NodeRefSet> ins, outs;
    NodeRefVec order;
    switch (dir) {
        case Direction::IN:
            po.postorder(std::back_inserter(order));
            break;
        case Direction::OUT:
            po.reverse_postorder(std::back_inserter(order));
            break;
    }
    
    for (NodeRef ref : order) {
        // in
        const NodeRefSet *preds_;
        switch (dir) {
            case Direction::IN:
                preds_ = &po.po.fwd.at(ref);
                break;
            case Direction::OUT:
                preds_ = &po.po.rev.at(ref);
                break;
        }
        const auto& preds = *preds_;
        NodeRefSet& in = ins[ref];
        for (auto it = preds.begin(); it != preds.end(); ++it) {
            const NodeRefSet& pred_out = outs.at(*it);
            if (it == preds.begin()) {
                in = pred_out;
            } else {
                NodeRefSet intersect;
                for (const NodeRef x : pred_out) {
                    if (in.find(x) != in.end()) {
                        intersect.insert(x);
                    }
                }
                in = std::move(intersect);
            }
        }
        
        // out
        NodeRefSet& out = outs[ref] = in;
        out.insert(ref);
    }
    
    // post-processing: compute dominator map
    DominatorMap doms;
    for (const auto& pair : outs) {
        for (const NodeRef dom : pair.second) {
            doms[dom].insert(pair.first);
        }
    }
    return doms;
}

void AEG::construct_ctrl() {
    /* Control dependencies are between loads of values used in computing
     * a branch condition and loads/stores within the branch.
     * While looking for leakage, we'll consider CTRL edges ending in a transiently executed instruction.
     *
     * At each program point, track the map of loads to the set of noderefs that depend on them.
     *
     * Only add a control dependency if it's actually in a branch... hmm.
     */
    
    /* Once we have the map of dependencies, how do we identify CTRL dependency?
     * For each branch, check the set of dependencies of the condition. Find loads.
     * The find memory accesses in the body of either branch, looking at
     *
     * Post-dominator. Control dependencies can only be from a dominator node to a node that has no intervening post-dominators.
     */
    
    // for each dominator, find the set of nodes it properly dominates (i.e. they don't postdominate it)
    DominatorMap excl_doms;
    for (const auto& dom_pair : dominators) {
        NodeRef dominator = dom_pair.first;
        for (NodeRef dominee : dom_pair.second) {
            const auto& postdom = postdominators.at(dominee);
            if (postdom.find(dominator) == postdom.end()) {
                excl_doms[dominator].insert(dominee);
            }
        }
    }
    
    /* For each branch, find dependencies of conditions that are loads. Then in set of exclusive postdominators, find memory accesses.
     */
    for (const NodeRef br_ref : node_range()) {
        const auto& br_node = lookup(br_ref);
        if (const auto *BI = llvm::dyn_cast_or_null<llvm::BranchInst>(br_node.inst->get_inst())) {
            for (const NodeRef load_dep_ref : dependencies.at(br_ref)) {
                const auto& load_dep_node = lookup(load_dep_ref);
                if (load_dep_node.may_read()) {
                    // find all memory accesses that the branch node dominates
                    // TODO: investigate whether this is expected or buggy
                    for (const NodeRef access_dom_ref : excl_doms[br_ref]) {
                        const Node& access_dom_node = lookup(access_dom_ref);
                        if (access_dom_node.may_access()) {
                            // EMIT EDGE
                            add_unidir_edge(load_dep_ref, access_dom_ref, Edge {Edge::CTRL, (load_dep_node.exec() && load_dep_node.read) && (br_node.exec()) && (access_dom_node.exec() && access_dom_node.access())});
                            std::cerr << "CTRL " << load_dep_ref << " " << access_dom_ref << "\n";
                        }
                    }
                }
            }
        }
    }
    
}



void AEG::construct_data() {
    /* DATA dependencies are syntactic dependencies from a load to a subsequent store, where the value operand of the store is computing using the value result of the load.
     */
    for (NodeRef store_ref : node_range()) {
        if (store_ref == entry) { continue; }
        const Node& store_node = lookup(store_ref);
        if (!store_node.may_write()) { continue; }
        const StoreInst *store_inst = dynamic_cast<const StoreInst *>(store_node.inst.get());
        if (store_inst == nullptr) { continue; }
        const llvm::Value *V = store_inst->get_value_operand();
        const auto& store_po_node = po.lookup(store_ref);
        const auto addr_refs_it = store_po_node.refs.find(V);
        if (addr_refs_it == store_po_node.refs.end()) { continue; }
        const auto& addr_refs = addr_refs_it->second;
        for (const NodeRef addr_ref : addr_refs) {
            NodeRefSet candidate_srcs = dependencies.at(addr_ref);
            candidate_srcs.insert(addr_ref);
            for (NodeRef candidate_src : candidate_srcs) {
                const Node& candidate_node = lookup(candidate_src);
                if (candidate_node.may_read()) {
                    add_unidir_edge(candidate_src, store_ref, Edge {
                        Edge::DATA,
                        (store_node.exec() && store_node.write) && (candidate_node.exec() && candidate_node.read)
                    });
                }
            }
        }
    }
}

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
            if (const auto *I = pred_node.inst->get_inst()) {
                if (const auto *B = llvm::dyn_cast<llvm::BranchInst>(pred_node.inst->get_inst())) {
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

void AEG::construct_com() {
    // initialize read, write
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        
        const auto f = [&] (Option o, const std::string& name) -> z3::expr {
            switch (o) {
                case Option::MUST: return context.TRUE;
                case Option::MAY: return context.make_bool(name);
                case Option::NO: return context.FALSE;
            }
        };
        
        node.read = f(node.inst->may_read(), "read");
        node.write = f(node.inst->may_write(), "write");
    }
}
