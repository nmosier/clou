#include "aeg.h"
#include "progress.h"
#include "timer.h"
#include "util/z3.h"
#include "taint.h"
#include "taint_bv.h"
#include "cfg/expanded.h"
#include "util/algorithm.h"
#include "util/iterator.h"
#include "util/output.h"
#include "config.h"

namespace aeg {

void AEG::construct(llvm::AliasAnalysis& AA, unsigned rob_size) {
    // initialize nodes
    std::transform(po.nodes.begin(), po.nodes.end(), std::back_inserter(nodes),
                   [&] (const CFG::Node& node) {
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

    logv(2) << "Constructing com\n";
    construct_com();

    logv(2) << "Constructing comx\n";
    construct_comx();
    logv(2) << "Constructing dependencies\n";

    dependencies = construct_dependencies2();

    logv(2) << "Constructing dominators\n";
    construct_dominators();
    logv(2) << "Constructing postdominators\n";
    construct_postdominators();
    logv(2) << "Constructing control-equivalents\n";
    construct_control_equivalents();
    
    // syntactic memory dependencies
    logv(2) << "Constructing addr\n";
    construct_addr();
#if 1
    logv(2) << "Constructing addr_gep\n";
    construct_addr_gep();
#endif
    logv(2) << "Constructing data\n";
    construct_data();
    logv(2) << "Constructing ctrl\n";
    construct_ctrl();
    
    if (partial_executions || stb_size) {
        compute_min_store_paths();
    }
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
#if 1
        for (NodeRef ref : order) {
            Node& node = lookup(ref);
            
            const auto& exec = po.execs.at(ref);
            const auto apply_exec = [&] (Option opt, const char *name) -> z3::expr {
#if 1
                switch (opt) {
                    case Option::MUST: return context.TRUE;
                    case Option::MAY: return context.make_bool(name);
                    case Option::NO: return context.FALSE;
                }
#else
                return context.make_bool(name);
#endif
            };
            
            node.arch = apply_exec(exec.arch, "arch");
            node.trans = apply_exec(exec.trans, "trans");
        }
#else
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
#endif
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
                node.addr_def = Address {context};
            }
        }
    }
}

void AEG::construct_addr_refs() {
    std::unordered_map<const llvm::Argument *, Address> main_args;
    std::unordered_map<const llvm::Constant *, Address> globals;
    
    for (NodeRef ref = 0; ref < size(); ++ref) {
        const CFG::Node& po_node = po.lookup(ref);
        Node& node = lookup(ref);
        
        if (const RegularInst *inst = dynamic_cast<const RegularInst *>(node.inst.get())) {
            
            for (const llvm::Value *V : inst->addr_refs) {
                const auto defs_it = po_node.refs.find(V);
                std::optional<Address> e;
                if (defs_it == po_node.refs.end()) {
                    if (const llvm::ConstantData *CD = llvm::dyn_cast<llvm::ConstantData>(V)) {
                        if (CD->isNullValue()) {
                            const auto zero = context.context.int_val(0);
                            e = Address {context.context.int_val(0)};
                        } else {
                            llvm::errs() << "unhandled constant data: " << *CD << "\n";
                            std::abort();
                        }
                    } else if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
                        auto main_args_it = main_args.find(A);
                        if (main_args_it == main_args.end()) {
                            main_args_it = main_args.emplace(A, Address {context}).first;
                        }
                        e = main_args_it->second;
                    } else if (const llvm::Constant *G = llvm::dyn_cast<llvm::Constant>(V)) {
                        auto globals_it = globals.find(G);
                        if (globals_it == globals.end()) {
                            globals_it = globals.emplace(G, Address {context}).first;
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
                        e = Address {context};
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
    
    const z3::expr_vector exit_archs = z3::transform(exits, [&] (const NodeRef ref) -> z3::expr {
        return lookup(ref).arch;
    });
    constraints(z3::exactly(exit_archs, 1), "exit-arch");
}

void AEG::construct_trans() {
    // NOTE: depends on results of construct_tfo()
    
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
    
    // ensure that the number of transiently executed nodes doesn't exceed trans limit
    {
        z3::expr_vector trans {context.context};
        for (NodeRef ref : node_range()) {
            trans.push_back(lookup(ref).trans);
        }
        unsigned max = num_specs();
        if (max_transient_nodes) {
            max = std::min(max, *max_transient_nodes);
        }
        constraints(z3::atmost(trans, max), "trans-limit-max");
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
    
    const auto edge_exists = [&] (const auto& edge) -> z3::expr {
        return edge->exists;
    };
    
    
    const auto count_func = partial_executions ? &z3::atmost : &z3::exactly;
    
    // add 'exactly one successor' constraint
    for (const NodeRef src : node_range()) {
        if (exits.find(src) != exits.end()) { continue; }
        const auto edges = get_edges(Direction::OUT, src, Edge::PO);
        const z3::expr_vector vec = z3::transform(edges, edge_exists);
        Node& src_node = lookup(src);
        src_node.constraints(z3::implies(src_node.arch, count_func(vec, 1)), "po-succ");
    }
    
    // add 'exactly one predecessor' constraint
    for (const NodeRef dst : node_range()) {
        if (dst == entry) { continue; }
        if (partial_executions && exits.find(dst) != exits.end()) { continue; }
        const auto edges = get_edges(Direction::IN, dst, Edge::PO);
        const z3::expr_vector vec = z3::transform(edges, edge_exists);
        Node& dst_node = lookup(dst);
        dst_node.constraints(z3::implies(dst_node.arch, count_func(vec, 1)), "po-pred");
    }
    
    if (partial_executions) {
        // only one cold start (predecessor with no po)
        const z3::expr_vector arch_intros = z3::transform(node_range(), [&] (const NodeRef ref) -> z3::expr {
            if (ref == entry) { return context.FALSE; }
            if (exits.find(ref) != exits.end()) { return context.FALSE; }
            const auto pos = get_edges(Direction::IN, ref, Edge::PO);
            const auto vec = z3::transform(context.context, pos, edge_exists);
            return !z3::implies(lookup(ref).arch, z3::mk_or(vec));
        });
        constraints(z3::exactly(arch_intros, 1), "exactly-1-cold-po-start");
    }
}

/// depends on construct_po()
void AEG::construct_tfo() {
    std::size_t nedges = 0;
    for (const NodeRef src : node_range()) {
        Node& src_node = lookup(src);
        z3::expr_vector tfos {context};
        for (const NodeRef dst : po.po.fwd.at(src)) {
            // add optional edge
            const Node& dst_node = lookup(dst);
            z3::expr_vector cond {context.context};
            if (po.may_introduce_speculation(src)) {
                cond.push_back(src_node.arch && dst_node.exec());
            }
            cond.push_back(src_node.trans && dst_node.trans);
            const z3::expr exists = add_optional_edge(src, dst, Edge {
                Edge::TFO,
                z3::mk_or(cond)
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
    
    // assert only one tfo window
    z3::expr_vector tfos {context.context};
    for_each_edge(Edge::TFO, [&] (const NodeRef src, const NodeRef dst, const Edge& edge) {
        const Node& src_node = lookup(src);
        const Node& dst_node = lookup(dst);
        tfos.push_back(src_node.arch && dst_node.trans && edge.exists);
    });
    constraints(z3::atmost(tfos, 1), "at-most-one-spec-intro");
    
    // entry has no po or tfo successors
    if (partial_executions) {
        for (const Edge::Kind kind : std::array<Edge::Kind, 2> {Edge::PO, Edge::TFO}) {
            const auto edges = get_nodes(Direction::OUT, entry, kind);
            const z3::expr_vector v = z3::transform(edges, [&] (const auto& p) -> z3::expr {
                return p.second;
            });
            lookup(entry).constraints(!z3::mk_or(v), util::to_string("entry-no-out-", kind));
        }
    }
    
    // exactly one arch with trans succ, no arch succ
    // TODO: could rephrase requirements more specifically
    if (partial_executions) {
        z3::expr_vector vec {context.context};
        for (const NodeRef ref : node_range()) {
            if (ref == entry) { continue; }
            
            const auto pos = get_nodes(Direction::OUT, ref, Edge::PO);
            const auto tfos = get_nodes(Direction::OUT, ref, Edge::TFO);
            
            const auto op = [] (const auto& p) -> z3::expr {
                return p.second;
            };
            const auto no_po = z3::exactly(z3::transform(context.context, pos, op), 0);
            const auto one_tfo = z3::exactly(z3::transform(context.context, tfos, op), 1);
            vec.push_back(lookup(ref).arch && no_po && one_tfo);
        }
        constraints(z3::exactly(vec, 1), "partial-po-end");
    }
}

std::optional<llvm::AliasResult> AEG::compute_alias(const AddrInfo& a, const AddrInfo& b, llvm::AliasAnalysis& AA) const {
    /* check if LLVM's built-in alias analysis is valid */
    if (po.llvm_alias_valid(a.id, b.id)) {
        return AA.alias(a.V, b.V);
    }
    
    if (alias_mode.llvm_only) {
        return std::nullopt;
    }
    
    /* check whether both values are allocated in nested scopes */
    {
        // if a is a prefix of b or b is a prefix of a
        if (util::prefix(a.id.func, b.id.func) || util::prefix(b.id.func, a.id.func)) {
            // check if both alloca
            if (llvm::isa<llvm::AllocaInst>(a.V) && llvm::isa<llvm::AllocaInst>(b.V)) {
                return llvm::AliasResult::NoAlias;
            }
            
            // EXPERIMENTAL: try inter-procedural alias analysis
            // return AA.alias(a.V, b.V);
        }
    }
    
    return std::nullopt;
}

void AEG::construct_aliases(llvm::AliasAnalysis& AA) {
    using ID = CFG::ID;
    
    std::vector<AddrInfo> addrs;
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
    unsigned nos, musts, mays, invalid;
    nos = musts = mays = invalid = 0;
    
#if 0
    // TODO: pointers are sketchy, but can't use iterators.
    using InfoSet = std::unordered_set<const Info *>;
    using InfoRel = std::unordered_map<const Info *, InfoSet>;
#endif
    
#if 0
    InfoRel no_map; // TODO: currently unused
#endif

#if 0
    using MustRel = std::unordered_map<ValueLoc, std::unordered_set<ValueLoc>>;
    MustRel must_rel;
#endif
    
    using ValueLocSet = std::unordered_set<ValueLoc>;
    ValueLocSet skip_vls; // skip because already saw 'must alias'
    
    for (auto it1 = addrs.begin(); it1 != addrs.end(); ++it1) {
        const ValueLoc vl1 = it1->vl();
        if (util::contains(skip_vls, vl1)) { continue; }
        for (auto it2 = std::next(it1); it2 != addrs.end(); ++it2) {
            if (const auto alias_res = compute_alias(*it1, *it2, AA)) {
                const ValueLoc vl2 = it2->vl();
                if (util::contains(skip_vls, vl2)) { continue; }

                const auto is_arch = [&] (const AddrInfo& x) -> z3::expr {
                    if (x.ref) {
                        return lookup(*x.ref).arch;
                    } else {
                        return context.TRUE;
                    }
                };
                
                const z3::expr arch1 = is_arch(*it1);
                const z3::expr arch2 = is_arch(*it2);
                const z3::expr precond = alias_mode.transient ? context.TRUE : (arch1 && arch2);
                
                switch (*alias_res) {
                    case llvm::AliasResult::NoAlias: {
#if 0
                        no_map[&*it1].insert(&*it2);
                        no_map[&*it2].insert(&*it1);
#endif
                        constraints(z3::implies(precond, it1->e != it2->e), "no-alias");
                        ++nos;
                        break;
                    }
                        
                    case llvm::AliasResult::MayAlias: {
                        ++mays;
                        break;
                    }
                        
                    case llvm::AliasResult::MustAlias: {
#if 0
                        must_rel[vl1].insert(vl2);
                        must_rel[vl2].insert(vl1);
#endif
                        skip_vls.insert(vl2);
                        constraints(z3::implies(precond, it1->e == it2->e), "must-alias");
                        ++musts;
                        break;
                    }
                        
                    default: std::abort();
                }
                
                add_alias_result(vl1, vl2, *alias_res);
                
            } else {
                ++invalid;
            }
        }
    }
    std::cerr << "NoAlias: " << nos << "\n"
    << "MustAlias: " << musts << "\n"
    << "MayAlias: " << mays << "\n"
    << "InvalidAlias: " << invalid << "\n";
    
#if 0
    // NOTE: This only improves assertions by 2x.
    std::vector<std::unordered_set<const Info *>> no_subgraphs;
    util::complete_subgraphs<const Info *>{no_map}(std::back_inserter(no_subgraphs));
    std::cerr << "NoAlias subgraphs: " << no_subgraphs.size() << "\n";
#endif
    
#if 0
    // analyze 'must alias'
    using ValueLocSet = std::unordered_set<ValueLoc>;
    std::vector<ValueLocSet> must_parts;
    util::complete_subgraphs<ValueLoc>{must_rel}(std::back_inserter(must_parts));
    std::cerr << "Must partitions: " << must_parts.size() << "\n";
    
    // get map from Info pointer to part
    std::unordered_map<ValueLoc, const ValueLocSet *> info_to_part;
    for (const Info& info : addrs) {
        const ValueLoc vl = info.vl();
        const auto it = std::find_if(must_parts.begin(), must_parts.end(), [&] (const ValueLocSet& set) -> bool {
            return static_cast<bool>(util::contains(set, vl));
        });
        assert(it != must_parts.end());
        info_to_part[vl] = &*it;
    }
    
    // rewrite alias rel to be minimal
    std::unordered_map<std::pair<const ValueLocSet *, const ValueLocSet *>, llvm::AliasResult> new_alias_rel;
    for (const auto& p : alias_rel) {
        const ValueLocSet *a = info_to_part.at(p.first.first);
        const ValueLocSet *b = info_to_part.at(p.first.second);
        new_alias_rel[std::make_pair(a, b)] = p.second;
    }
    
    // now add assertions
    
#endif
    
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
    
    // connect xstate and addr
    const bool psf = g_psf();
    for (NodeRef ref : xsaccesses) {
        Node& node = lookup(ref);
        
        // sibling instructions always have the same address
        
        
        if (g_psf()) {
            
        } else {
            
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
    constraints(z3::distinct2(vec), "xsaccess-order-distinct");
#endif
}


template <typename Func>
void AEG::for_each_dependency(NodeRef ref, const llvm::Value *V, Func func) {
    const auto& refs = po.lookup(ref).refs;
    const auto it = refs.find(V);
    if (it == refs.end()) { return; }
    for (const NodeRef ref_ref : it->second) {
        NodeRefSet deps = dependencies.at(ref_ref);
        deps.insert(ref_ref);
        for (NodeRef dep : deps) {
            func(dep);
        }
    }
}

void AEG::construct_addr() {
    /* Address dependencies are from a load to a subsequent access.
     * The address of the access should be dependent on the result of the load.
     * This means that the address operand of the access instruction should be the load or list it as a dependency.
     */
    
    for (NodeRef dst : node_range()) {
        const Node& dst_node = lookup(dst);
        if (!dst_node.may_access()) { continue; }
        const MemoryInst *dst_inst = dynamic_cast<const MemoryInst *>(dst_node.inst.get());
        if (dst_inst == nullptr) { continue; }
        const llvm::Value *dst_addr = dst_inst->get_memory_operand();
        for_each_dependency(dst, dst_addr, [&] (const NodeRef src) {
            const Node& src_node = lookup(src);
            if (!src_node.may_read()) { return; }
            add_unidir_edge(src, dst, Edge {
                Edge::ADDR,
                (src_node.exec() && src_node.read) && (dst_node.exec() && dst_node.access())
            });
        });
    }
}

bool construct_addr_gep_nonconst(const llvm::Value *V) {
    if (llvm::isa<llvm::Instruction>(V)) {
        return true;
    } else if (llvm::isa<llvm::Argument>(V)) {
        return true;
    } else if (llvm::isa<llvm::Constant>(V)) {
        if (llvm::isa<llvm::ConstantData>(V)) {
            return false;
        } else if (llvm::isa<llvm::ConstantExpr>(V)) {
            return false;
        } else if (llvm::isa<llvm::GlobalValue>(V)) {
            return true;
        } else if (llvm::isa<llvm::BlockAddress>(V)) {
            return true;
        }
    }
    llvm::errs() << __FUNCTION__ << ": couldn't categorize as (non)const: " << *V << "\n";
    std::abort();
}

void AEG::construct_addr_gep() {
    
    std::unordered_map<NodeRefPair, z3::expr> edges;
    
    for (NodeRef dst : node_range()) {
        const Node& dst_node = lookup(dst);
        if (!dst_node.may_access()) { continue; }
        const MemoryInst *dst_inst = dynamic_cast<const MemoryInst *>(dst_node.inst.get());
        if (dst_inst == nullptr) { continue; }
        const llvm::Value *dst_addr = dst_inst->get_memory_operand();
        for_each_dependency(dst, dst_addr, [&] (const NodeRef gep) {
            // gep must be a GetElementPtrInst instruction
            const Node& gep_node = lookup(gep);
            const llvm::GetElementPtrInst *gep_I = llvm::dyn_cast_or_null<llvm::GetElementPtrInst>(gep_node.inst->get_inst());
            if (gep_I == nullptr) { return; }
            
            for (const llvm::Value *gep_idx : gep_I->indices()) {
                for_each_dependency(gep, gep_idx, [&] (const NodeRef src) {
                    // src must be load
                    const Node& src_node = lookup(src);
                    if (!src_node.may_read()) { return; }
                    
                    // edges
                    const z3::expr cond = (src_node.exec() && src_node.read) && (gep_node.exec()) && (dst_node.exec() && dst_node.access());
                    
                    z3::expr& val = edges.emplace(NodeRefPair(src, dst), context.FALSE).first->second;
                    val = val || cond;
                });
            }
        });
    }
    
    for (const auto& p : edges) {
        add_unidir_edge(p.first.first, p.first.second, Edge {
            Edge::ADDR_GEP,
            p.second
        });
    }
}

#if 0
void AEG::construct_addr_gep() {
    /* For each possible dst (access), find a GEP on which the dst memory addr depends.
     * src -dep-> GEP -dep-> dst,
     * where src is read and dst is access
     */
    
    for (const NodeRef dst : node_range()) {
        // dst must be access
        const Node& dst_node = lookup(dst);
        if (!dst_node.may_access()) { continue; }
        
        // dst must be memory instruction
        const MemoryInst *dst_inst = dynamic_cast<const MemoryInst *>(dst_node.inst.get());
        if (dst_inst == nullptr) { continue; }
        
        // get dependencies of dst's memory operand
        const llvm::Value *dst_addr = dst_inst->get_memory_operand();
        const auto& dst_refs = po.lookup(dst).refs;
        const auto dst_addr_refs_it = dst_refs.find(dst_addr);
        if (dst_addr_refs_it == dst_refs.end()) { continue; }
        const auto& dst_addr_refs = dst_addr_refs_it->second;
        
        // iterate over GEP candidates
        for (const NodeRef gep : dst_addr_refs) {
            // gep must be GetElementPtr instruction
            const llvm::Instruction *gep_I = lookup(gep).inst->get_inst();
            if (!llvm::isa<llvm::GetElementPtrInst>(gep_I)) { continue; }
            
            // iterate over src candidates
            depe
        }
    }
    
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
#endif

// TODO: rewrite in space-efficient way?
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

AEG::DependencyMap AEG::construct_dependencies2() {
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    DependencyMap map;
    for (const NodeRef dst : order) {
        const CFG::Node& node = po.lookup(dst);
        NodeRefSet& out_set = map[dst];
        for (const auto& ref_pair : node.refs) {
            for (const NodeRef ref_ref : ref_pair.second) {
                out_set.insert(ref_ref);
                util::copy(map.at(ref_ref), std::inserter(out_set, out_set.end()));
            }
        }
    }
    
    return map;
}

AEG::DominatorMap AEG::construct_dominators_shared(Direction dir) const {
    /* At each program point, store the set of instructions that MUST have been executed to reach this instruction. This means that the MEET operator is set intersection.
     */
    std::unordered_map<NodeRef, NodeRefBitset> ins, outs;
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
        NodeRefBitset& in = ins[ref];
        for (auto it = preds.begin(); it != preds.end(); ++it) {
            const NodeRefBitset& pred_out = outs.at(*it);
            if (it == preds.begin()) {
                in = pred_out;
            } else {
#if 0
                NodeRefBitset intersect;
                for (const NodeRef x : pred_out) {
                    if (in.find(x) != in.end()) {
                        intersect.insert(x);
                    }
                }
                in = std::move(intersect);
#else
                in &= pred_out;
#endif
            }
        }
        
        // out
        NodeRefBitset& out = outs[ref] = in;
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

void AEG::construct_control_equivalents() {
    // depends on AEG::construct_dominators(), AEG::construct_postdominators()
    /* Find all node pairs that have each other as dominator/postdominator.
     * Brute force: O(n^2)
     */
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    for (auto it1 = order.begin(); it1 != order.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != order.end(); ++it2) {
            if (util::contains(postdominators.at(*it1), *it2) && util::contains(dominators.at(*it2), *it1)) {
                control_equivalents[*it2].insert(*it1);
            }
        }
    }
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

void AEG::compute_min_store_paths() {
    assert(partial_executions);
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    for (const NodeRef ref : order) {
        const NodeRefSet& preds = po.po.rev.at(ref);
        const unsigned min = std::transform_reduce(preds.begin(), preds.end(), std::numeric_limits<unsigned>::max(), [] (unsigned a, unsigned b) -> unsigned {
            return std::min(a, b);
        }, [&] (const NodeRef ref) -> unsigned {
            return lookup(ref).stores_out;
        });
        Node& node = lookup(ref);
        node.stores_out = node.stores_in = min;
        if (node.read.is_true()) {
            ++node.stores_out;
        }
    }
    
    std::cerr << __FUNCTION__ << ": " << size() << " nodes, min stores at exits:";
    for (const NodeRef exit : exits) {
        std::cerr << " " << lookup(exit).stores_out;
    }
    std::cerr << "\n";
}


}
