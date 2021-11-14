#include "aeg.h"
#include "progress.h"
#include "timer.h"
#include "util/z3.h"
#include "cfg/expanded.h"
#include "util/algorithm.h"
#include "util/iterator.h"
#include "util/output.h"
#include "config.h"
#include "cfg/block.h"
#include "util/llvm.h"

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
    logv(2, "Number of loads: " << count_kind(Inst::Kind::LOAD) << "\n");
    logv(2, "Number of stores: " << count_kind(Inst::Kind::STORE) << "\n");
    
    
    logv(2, "Constructing nodes\n");
    construct_nodes();
    logv(2, "Construct arch\n");
    construct_arch();
    logv(2, "Constructing tfo\n");
    construct_tfo();
    logv(2, "Constructing exec\n");
    construct_exec();
    logv(2, "Constructing addr defs\n");
    construct_addr_defs();
    logv(2, "Constructing addr refs\n");
    construct_addr_refs();
    logv(2, "Constructing aliases\n");
    construct_aliases(AA);

    logv(2, "Constructing com\n");
    construct_com();

    logv(2, "Constructing comx\n");
    construct_comx();
    logv(2, "Constructing dependencies\n");

    dependencies = construct_dependencies2();

    logv(2, "Constructing dominators\n");
    construct_dominators();
    logv(2, "Constructing postdominators\n");
    construct_postdominators();
    logv(2, "Constructing control-equivalents\n");
    construct_control_equivalents();
    
    // syntactic memory dependencies
    logv(2, "Constructing addr\n");
    construct_addr();
    logv(2, "Constructing addr_gep\n");
    construct_addr_gep();
    logv(2, "Constructing data\n");
    construct_data();
    logv(2, "Constructing ctrl\n");
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
    for (NodeRef ref : order) {
        Node& node = lookup(ref);
        if (ref == entry || exits.find(ref) != exits.end()) {
            node.trans = context.FALSE;
        } else {
            node.trans = context.make_bool("trans");
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
    unsigned stack_counter = 1;
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    for (NodeRef ref : order) {
        Node& node = lookup(ref);
        if (auto *RI = dynamic_cast<RegularInst *>(node.inst.get())) {
            // TODO: this is fragmented. Try to unify addr_defs
            if (RI->addr_def) {
                
                if (const llvm::AllocaInst *AI = llvm::dyn_cast<llvm::AllocaInst>(RI->get_inst())) {
                    // TODO: lift out datalayout
                    llvm::DataLayout layout {AI->getParent()->getParent()->getParent()};
                    node.addr_def = Address(context.context.int_val(stack_counter));
                    stack_counter += layout.getTypeSizeInBits(AI->getType());
                    continue;
                }
                
                if (const llvm::GetElementPtrInst *GEP = llvm::dyn_cast<llvm::GetElementPtrInst>(RI->get_inst())) {
                    if (const auto offset = llvm::getelementptr_const_offset(GEP)) {
                        if (const llvm::Instruction *I = llvm::dyn_cast<llvm::Instruction>(GEP->getPointerOperand())) {
                            const auto& refs = po.lookup(ref).refs.at(I);
                            if (refs.size() == 1) {
                                const NodeRef base_ref = *refs.begin();
                                node.addr_def = Address((lookup(base_ref).addr_def->addr + *offset).simplify());
#if 0
                                llvm::errs() << "GEP address: " << util::to_string(node.addr_def->addr) << ": " << *GEP << "\n";
#endif
                                continue;
                            }
                        }
                    }
                }
                
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
#if 0
                        llvm::errs() << "GLOBAL: " << *G << "\n" << inst->I << "\n";
#endif
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
                            using output::operator<<;
                            std::stringstream desc;
                            desc << "addr-ref-" << ref << "-" << defs;
                            node.constraints(util::any_of<z3::expr>(defs.begin(), defs.end(),
                                                                    [&] (NodeRef def) {
                                return lookup_def(def) == *e;
                            }, context.FALSE), desc.str().c_str());
                        }
                    }
                }
                node.addr_refs.emplace(V, *e);
            }
        }
    }
}


#if 0
void AEG::construct_arch_po() {
    /* entry */
    lookup(entry).arch = context.context.bool_val(true);
    
    /* exit */
    {
        z3::expr_vector enables {context.context};
        for (NodeRef ref : exits) {
            const z3::expr enable = context.context.bool_val(util::to_string("arch-exit-enable-", ref).c_str());
            enables.push_back(enable);
            lookup(ref).arch = enable;
        }
        constraints(z3::exactly(enables, 1), "one-arch-exit");
    }
    
    /* other */
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    z3::expr_vector enables {context.context}; // this holds the enable booleans
    for (NodeRef ref : order) {
        if (ref == entry || exits.find(ref) != exits.end()) { continue; }
        
        Node& node = lookup(ref);
        
        /* collect incoming po edges; set arch */
        const z3::expr enable = context.context.bool_const(util::to_string("arch-enable-", ref).c_str());
        enables.push_back(enable);
        z3::expr_vector arch_v {context.context};
        arch_v.push_back(enable);
        for (const auto& po_in : get_edges(Direction::IN, ref, Edge::PO)) {
            arch_v.push_back(po_in->exists);
        }
        node.arch = z3::mk_or(arch_v);
        
        /* create outgoing po edges */
        EdgeTupleVec po_outs;
        for (NodeRef dst : po.po.fwd.at(ref)) {
            po_outs.emplace_back(ref, dst, Edge(Edge::PO, node.arch));
        }
        add_mux_edge(po_outs, util::to_string("po-mux-", ref));
    }
    
    constraints(z3::exactly(enables, 1), "one-arch-cold-start");
}

void AEG::construct_trans_tfo() {
    /* entry */
    lookup(entry).trans = context.context.bool_val(false);
    
    /* exits */
    for (NodeRef ref : exits) {
        lookup(ref).trans = context.context.bool_val(false);
    }
    
    /* other */
    
    /* compute min distance to speculation gadget */
    std::unordered_map<NodeRef, unsigned> min_specs_in, min_specs_out;
    if (max_transient_nodes) {
        NodeRefVec order;
        po.reverse_postorder(std::back_inserter(order));
                
        for (NodeRef ref : order) {
            const auto& preds = po.po.rev.at(ref);
            unsigned min = std::transform_reduce(preds.begin(), preds.end(), *max_transient_nodes, [] (unsigned a, unsigned b) -> unsigned {
                return std::min(a, b);
            }, [&] (const NodeRef ref) -> unsigned {
                return min_specs_out.at(ref);
            });

            min_specs_in.emplace(ref, min);
            
            if (po.may_introduce_speculation(ref)) {
                min = 0;
            } else {
                min = std::min(*max_transient_nodes, min + 1);
            }
            
            min_specs_out.emplace(ref, min);
        }
    }
    
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    z3::expr_vector transes {context.context};
    for (NodeRef ref : order) {
        if (ref == entry || exits.find(ref) != exits.end()) { continue; }
        
        Node& node = lookup(ref);
        
        /* check if can even be transiently executed */
        if (max_transient_nodes && min_specs_in.at(ref) >= *max_transient_nodes) {
            std::cerr << "pruning " << ref << "\n";
            node.trans = context.context.bool_val(false);
            continue;
        }
        
        /* collect incoming tfo edges */
        z3::expr_vector trans_v {context.context};
        for (const auto& tfo_in : get_edges(Direction::IN, ref, Edge::TFO)) {
            trans_v.push_back(tfo_in->exists);
        }
        node.trans = z3::mk_or(trans_v);
        transes.push_back(node.trans);
        
        /* assert arch/trans exclusivity */
        node.constraints(!(node.arch && node.trans), util::to_string("arch-trans-exclusive-", ref).c_str());
        
        const auto po_out = get_edges(Direction::OUT, ref, Edge::PO);
        const auto no_po_out = !z3::mk_or(z3::transform(context.context, po_out, [] (const auto& e) -> z3::expr {
            return e->exists;
        }));
        
        /* create outgoing tfo edges */
        EdgeTupleVec tfo_outs;
        for (NodeRef dst : po.po.fwd.at(ref)) {
#if 1
            const z3::expr tfo_exists = context.context.bool_const(util::to_string("tfo-", ref, "-", dst).c_str());
            node.constraints(tfo_exists == (node.trans || (po.may_introduce_speculation(ref) && node.arch && no_po_out)), util::to_string("tfo-def-", ref, "-", dst).c_str());
            tfo_outs.emplace_back(ref, dst, Edge {
                Edge::TFO,
                tfo_exists
            });
#else
            tfo_outs.emplace_back(ref, dst, Edge {
                Edge::TFO,
                node.trans || (po.may_introduce_speculation(ref) && node.arch && no_po_out)
            });
#endif
        }
        add_mux_edge(tfo_outs, util::to_string("tfo-mux-", ref));
        
    }
    
    /* trans limit */
    if (max_transient_nodes) {
        constraints(z3::atmost(transes, *max_transient_nodes), "max-transient-nodes");
    }
    
}
#endif


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
    
    // construct_arch();
    construct_trans();
}
 
void AEG::construct_arch() {
    // assign arch variables
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    for (NodeRef ref : order) {
        z3::expr& arch = lookup(ref).arch;
        if (ref == entry) {
            arch = context.TRUE;
        } else {
            arch = context.make_bool("arch");
        }
    }
    
    // one exit arch
    constraints(z3::exactly(z3::transform(exits, [&] (NodeRef ref) -> z3::expr {
        return lookup(ref).arch;
    }), 1), "one-exit-arch");
}


void AEG::construct_arch_po() {
    /* entry */
    lookup(entry).arch = context.context.bool_val(true);
    
    /* exits */
    {
        z3::expr_vector exit_archs {context.context};
        for (NodeRef ref : exits) {
            z3::expr arch = context.make_bool("arch");
            lookup(ref).arch = arch;
            exit_archs.push_back(arch);
        }
        constraints(z3::exactly(exit_archs, 1), "one-exit-arch");
    }
    
    /* other */
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    z3::expr_vector arch_enables {context.context};
    for (NodeRef ref : order) {
        if (ref == entry || exits.find(ref) != exits.end()) { continue; }
        
        Node& node = lookup(ref);
        node.arch = context.make_bool("arch");
        
        /* collect incoming po edges */
        const auto po_ins = z3::transform(context.context, get_edges(Direction::IN, ref, Edge::PO), [] (const auto& e) -> z3::expr {
            return e->exists;
        });
        node.constraints(z3::atmost2(po_ins, 1), "po-in-limit");
        arch_enables.push_back(node.arch && !z3::mk_or(po_ins)); // cold start
        
        /* create outgoing po edges */
        // NOTE: Only if no arch successors.
        z3::expr_vector po_outs {context.context};
        for (NodeRef dst : po.po.fwd.at(ref)) {
            const z3::expr exists = add_optional_edge(ref, dst, Edge {
                Edge::PO,
                node.arch
            }, "po");
            po_outs.push_back(exists);
        }
        node.constraints(z3::atmost2(po_outs, 1), "po-out-limit");
    }
}


void AEG::construct_trans() {
    // NOTE: depends on results of construct_tfo()
    
    // transient execution of node requires incoming tfo edge
    for (const auto ref : node_range()) {
        Node& node = lookup(ref);
        const auto tfos = get_edges(Direction::IN, ref, Edge::TFO);
        // TODO: experiment with using z3::{atleast,exactly}(vec, 1) instead.
        const auto tfo_vec = z3::transform(context.context, tfos, [] (const auto& edge) -> z3::expr {
            return edge->exists;
        });
        const auto f = z3::exactly(tfo_vec, 1);
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
    
    // calculate min distance to speculation gadget
    if (max_transient_nodes) {
        NodeRefVec order;
        po.reverse_postorder(std::back_inserter(order));
        
        std::unordered_map<NodeRef, unsigned> min_specs_in, min_specs_out;
        for (NodeRef ref : order) {
            const auto& preds = po.po.rev.at(ref);
            unsigned min = std::transform_reduce(preds.begin(), preds.end(), *max_transient_nodes, [] (unsigned a, unsigned b) -> unsigned {
                return std::min(a, b);
            }, [&] (const NodeRef ref) -> unsigned {
                return min_specs_out.at(ref);
            });

            min_specs_in.emplace(ref, min);
            
            if (min >= *max_transient_nodes) {
                lookup(ref).trans = context.FALSE;
            }
            
            if (po.may_introduce_speculation(ref)) {
                min = 0;
            } else {
                min = std::min(*max_transient_nodes, min + 1);
            }
            
            min_specs_out.emplace(ref, min);
        }
    }
} 

void AEG::construct_po() {
    logv(3, __FUNCTION__ << ": adding edges\n");
    
#if 0
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
#else
    for (const NodeRef src : node_range()) {
        Node& src_node = lookup(src);
        
        // add successor po edges
        z3::expr_vector enables {context.context};
        for (const NodeRef dst : po.po.fwd.at(src)) {
            Edge edge {
                Edge::PO,
                src_node.arch && lookup(dst).arch
            };
            const z3::expr enable = context.make_bool(util::to_string("po-enable-", src, "-", dst));
            edge.exists = enable && edge.exists;
            add_unidir_edge(src, dst, edge);
            enables.push_back(enable);
        }
        
        src_node.constraints(z3::atmost2(enables, 1), util::to_string("po-enable-one-", src).c_str());
    }
#endif
    
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
    NodeRefVec order;
    po.reverse_postorder(std::back_inserter(order));
    
    std::size_t nedges = 0;
    for (const NodeRef src : node_range()) {
        if (src == entry || exits.find(src) != exits.end()) { continue; }
        
        Node& src_node = lookup(src);
        z3::expr_vector tfos {context};
        for (const NodeRef dst : po.po.fwd.at(src)) {
            // add optional edge
            const Node& dst_node = lookup(dst);
            z3::expr_vector cond {context.context};
            cond.push_back(src_node.arch && dst_node.arch);
            if (po.may_introduce_speculation(src)) {
                cond.push_back(src_node.arch && dst_node.trans);
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
    constraints(z3::atmost2(tfos, 1), "at-most-one-spec-intro");
    
    // if node introduces speculation, it has no arch successor in tfo
    if (partial_executions) {
        for (const NodeRef ref : node_range()) {
            const auto tfos = get_nodes(Direction::OUT, ref, Edge::TFO);
            const auto some_trans_succ = z3::mk_or(z3::transform(context.context, tfos, [&] (const auto& p) -> z3::expr {
                return p.second && lookup(p.first).trans;
            }));
            const auto no_arch_succ = z3::mk_or(z3::transform(context.context, tfos, [&] (const auto& p) -> z3::expr {
                return p.second && lookup(p.first).arch;
            }));
#if 0
            Node& node = lookup(ref);
            node.constraints(z3::implies(node.arch && some_trans_succ, no_arch_succ), "speculation-arch-no-po");
#endif
        }
    }
    
    // only one cold arch start
    z3::expr_vector cold_start {context.context};
    for (NodeRef ref : order) {
        if (ref == entry || exits.find(ref) != exits.end()) { continue; }
        const Node& node = lookup(ref);
        const auto tfo_ins = get_edges(Direction::IN, ref, Edge::TFO);
        const auto tfo_ins_v = z3::transform(context.context, tfo_ins, [] (const auto& e) -> z3::expr { return e->exists; });
        cold_start.push_back(node.arch && !z3::mk_or(tfo_ins_v));
    }
    constraints(z3::exactly(cold_start, 1), "one-cold-start");
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
    
    logv(3, "constructing xsaccess order...\n");
    construct_xsaccess_order(xsaccesses);
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
        Node& node = lookup(ref);
        
        if (ref == entry) {
            
            node.stores_out = 0;
            
        } else {
            
            const NodeRefSet& preds = po.po.rev.at(ref);
            const auto min = std::transform_reduce(preds.begin(), preds.end(), std::numeric_limits<decltype(node.stores_in)>::max(), [] (auto a, auto b) {
                return std::min(a, b);
            }, [&] (const NodeRef ref) {
                return lookup(ref).stores_out;
            });
            node.stores_out = node.stores_in = min;
            if (node.read.is_true()) {
                ++node.stores_out;
            }
            
        }
    }
    
    Node& entry_node = lookup(entry);
    entry_node.stores_in = entry_node.stores_out = std::numeric_limits<decltype(entry_node.stores_in)>::min();
    
    std::cerr << __FUNCTION__ << ": " << size() << " nodes, min stores at exits:";
    for (const NodeRef exit : exits) {
        std::cerr << " " << lookup(exit).stores_out;
    }
    std::cerr << "\n";
}


}
