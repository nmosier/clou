#include "aeg.h"
#include "progress.h"
#include "timer.h"

void AEG::construct(unsigned spec_depth, llvm::AliasAnalysis& AA) {
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
    construct_tfo();
    logv(2) << "Constructing exec\n";
    construct_exec();
    logv(2) << "Constructing addr defs\n";
    construct_addr_defs();
    logv(2) << "Constructing addr refs\n";
    construct_addr_refs();
#if 1
    logv(2) << "Constructing aliases\n";
    construct_aliases(AA);
#endif
    logv(2) << "Constructing com\n";
    construct_com();
#if 1
    logv(2) << "Constructing exec order\n";
    construct_exec_order();
    logv(2) << "Constructing trans group\n";
    construct_trans_group();
#endif
    
    // construct_mem();
    
#if 1
    logv(2) << "Constructing comx\n";
    construct_comx();
#endif
    logv(2) << "Constructing addr\n";
    construct_addr();
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
    
    // initialize `exec_order`
    {
        for (NodeRef ref : order) {
            Node& node = lookup(ref);
            const auto& preds = po.po.rev.at(ref);
            const auto it = std::max_element(preds.begin(), preds.end(), std::less<unsigned>());
            node.arch_order = it == preds.end() ? 0 : *it + 1;
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
                        e = UHBAddress {zero, zero};
                        // addr.arch = addr.trans = context.context.int_val(CD->getUniqueInteger().getLimitedValue());
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
                            return lookup_def(def).arch == e->arch;
                        }, context.FALSE), "addr-ref-arch");
                        node.constraints(util::any_of<z3::expr>(defs.begin(), defs.end(),
                                                                [&] (NodeRef def) {
                            return lookup_def(def).trans == e->trans;
                        }, context.FALSE), "addr-ref-trans");
                    }
                }
            }
            node.addr_refs.emplace(V, *e);
        }
    }
}

void AEG::construct_exec() {
    // NOTE: depends on results of construct_tfo().
    
    // exclusive architectural/transient execution
    for (Node& node : nodes) {
        node.constraints(!(node.arch && node.trans), "excl-exec");
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
    for (const auto v : node_range2()) {
        const auto& preds = po.po.rev.at(v.ref);
        if (preds.size() == 1) {
            const NodeRef pred = *preds.begin();
            const auto f = z3::implies(v.node.trans, v.node.trans_depth == lookup(pred).trans_depth + context.context.int_val(1));
            v.node.constraints(f, "depth-add");
        }
    }
    
    // transient execution of node requires incoming tfo edge
    for (const auto v : node_range2()) {
        const auto tfos = get_edges(Direction::IN, v.ref, Edge::TFO);
        const z3::expr f = util::any_of(tfos.begin(), tfos.end(), [] (const auto& edge) -> z3::expr {
            return edge->exists;
        }, context.FALSE);
        v.node.constraints(z3::implies(v.node.trans, f), "trans-tfo");
    }
}

void AEG::construct_po() {
#if 0
    // add edges
    for (NodeRef src : node_range()) {
        // TODO: This can be simplified if there's only one successor
        for (NodeRef dst : po.po.fwd.at(src)) {
            add_unidir_edge(src, dst, Edge {Edge::PO, context, "po"});
        }
    }
    
    // constrain edges
    for (const auto v : node_range2()) {
        const auto constrain = [&] (Direction dir, NodeRef exclude) {
            if (v.ref != exclude) {
                const auto edges = get_edges(dir, v.ref, Edge::PO);
                assert(!edges.empty());
                const auto f = util::one_of(edges, [] (const auto& edge) -> z3::expr {
                    return edge->exists;
                }, context.TRUE, context.FALSE);
                v.node.constraints(z3::implies(v.node.arch, f), std::string("po-") + util::to_string(dir));
            }
        };
        constrain(Direction::IN, entry);
        constrain(Direction::OUT, exit);
    }
    
    graph.for_each_edge([&] (NodeRef src, NodeRef dst, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(src).arch && lookup(dst).arch), "po-exec");
    });
#endif
}

void AEG::construct_tfo() {
    /* Consider multiple cases for tfo destination nodes.
     * If all predecessors have one successor, then that is the exact set.
     *
     * Condider each (pred, dst) pair in turn.
     * If the predecessor has one successor, then just add the tfo edge (pred, dst).
     * If the predecessor has multiple successors, add the tfo edge (pred, dst) but also add tfo edges from other successors of the predecessor up until a node has multiple predecessors.
     */
    Progress progress;
    
    logv(3) << __FUNCTION__ << ": adding edges\n";
    std::size_t n_fwd_edges = 0, n_rev_edges = 0;
    for (const NodeRef dst : node_range()) {
        const Node& dst_node = lookup(dst);
        for (const NodeRef src : po.po.rev.at(dst)) {
            const Node& src_node = lookup(src);
            
            z3::expr cond = context.FALSE;
            
            // (arch, arch)
            cond = cond || (src_node.arch && dst_node.arch);
            
            // (arch, trans)
            cond = cond || (src_node.arch && dst_node.trans && po.po.fwd.at(src).size() > 1);
            
            // (trans, trans)
            cond = cond || (src_node.trans && dst_node.trans);
            
            add_optional_edge(src, dst, Edge {Edge::TFO, cond});
            ++n_fwd_edges;
            
            
            const auto& succs = po.po.fwd.at(src);
            std::vector<std::pair<NodeRef, unsigned>> todo;
            std::unordered_map<NodeRef, unsigned> seen;
            std::transform(succs.begin(), succs.end(), std::back_inserter(todo), [] (NodeRef ref) {
                return std::make_pair(ref, 1U);
            });
            while (!todo.empty()) {
                const auto pair = todo.back();
                const NodeRef ref = pair.first;
                const auto spec_depth = pair.second;
                todo.pop_back();
                
                if (spec_depth > num_specs()) {
                    continue;
                }
                
                const auto seen_it = seen.find(ref);
                if (seen_it != seen.end() && seen_it->second < spec_depth) {
                    continue;
                }
                
                seen[ref] = spec_depth;
                
                const auto& ref_preds = po.po.rev.at(ref);
                if (ref_preds.size() == 1) {
                    const Node& ref_node = lookup(ref);
                    add_optional_edge(ref, dst, Edge {Edge::TFO, ref_node.trans && dst_node.arch});
                    ++n_rev_edges;
                    const auto& ref_succs = po.po.fwd.at(ref);
                    std::transform(ref_succs.begin(), ref_succs.end(), std::back_inserter(todo), [&] (NodeRef ref) {
                        return std::make_pair(ref, spec_depth + 1);
                    });
                }
            }
        }
    }
    logv(3) << __FUNCTION__ << ": added " << n_fwd_edges << " forward edges and " << n_rev_edges << " backward edges\n";
    
    // exactly one predecessor
    logv(3) << __FUNCTION__ << ": adding constraint 'exactly one predecessor'\n";
    progress = Progress(size());
    for (const NodeRef dst : node_range()) {
        if (dst != entry) {
            const auto tfos = get_edges(Direction::IN, dst, Edge::TFO);
            const auto f = util::one_of(tfos.begin(), tfos.end(), [] (const auto& edge) {
                return edge->exists;
            }, context.TRUE, context.FALSE);
            Node& node = lookup(dst);
            node.constraints(z3::implies(node.get_exec(), f), "tfo-pred");
            
            /* one of improvement
             * P => L ^ R
             * If true, then one half is true
             */
        }
        ++progress;
    }
    progress.done();
    
    // exactly one successor
    logv(3) << __FUNCTION__ << ": adding constraint 'exactly one successor'\n";
    for (const NodeRef ref : node_range()) {
        if (exits.find(ref) == exits.end()) {
            const auto tfos = get_edges(Direction::OUT, ref, Edge::TFO);
            const auto f = util::one_of(tfos.begin(), tfos.end(), [] (const auto &edge) {
                return edge->exists;
            }, context.TRUE, context.FALSE);
            Node& node = lookup(ref);
            node.constraints(z3::implies(node.get_exec(), f), "tfo-succ");
        }
    }
}

void AEG::construct_aliases(llvm::AliasAnalysis& AA) {
    using ID = AEGPO::ID;
    struct Info {
        ID id;
        const llvm::Value *V;
        z3::expr e;
    };
    std::vector<Info> addrs;
    std::unordered_map<std::pair<ID, const llvm::Value *>, NodeRef> seen;
    for (NodeRef i = 0; i < size(); ++i) {
        const Node& node = lookup(i);
        if (node.addr_def) {
            const ID& id = *po.lookup(i).id;
            const llvm::Value *V = node.inst.I;
            addrs.push_back({id, V, node.addr_def->arch});
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
                    addrs.push_back({id, V, it->second.arch});
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
                switch (alias_res) {
                    case llvm::NoAlias:
                        constraints(it1->e != it2->e, "no-alias");
                        ++nos;
                        break;
                    case llvm::MayAlias:
                        ++mays;
                        break;
                    case llvm::MustAlias:
                        constraints(it1->e == it2->e, "must-alias");
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

void AEG::construct_rf(const NodeRefVec& reads, const NodeRefVec& writes,
                       const ClosureMap& pred_reads, const ClosureMap& pred_writes) {
    constexpr bool construct_directly = true;
    
    Progress progress;
    
    // add edges
    logv(3) << __FUNCTION__ << ": adding edges\n";
    unsigned nedges = 0;
    unsigned skipped = 0;
    if constexpr (!construct_directly) {
        for (const NodeRef read : reads) {
            const auto& sourced_writes = pred_writes.at(read);
            for (const NodeRef write : sourced_writes) {
                switch (check_alias(read, write)) {
                    case llvm::MustAlias:
                        // TODO: This should be handled as separate case.
                    case llvm::MayAlias:
                        add_optional_edge(write, read, Edge {Edge::RF, context.TRUE}, "rf");
                        break;
                    case llvm::NoAlias:
                        ++skipped;
                        break;
                    default:
                        std::abort();
                }
                ++nedges;
            }
        }
    } else {
        // do a dataflow analysis checking which writes could've been sourced.
        struct Answer {
            z3::expr yes, no;
        };
        const Answer default_answer {context.FALSE, context.TRUE};
        using Value = std::unordered_map<NodeRef, Answer>;
        using Analysis = Dataflow<Value>;
        Analysis::Result res;
        const auto transfer = [&] (NodeRef src, const Value& in) -> Value {
            const Node& src_node = lookup(src);
            if (src_node.is_write()) {
                Value out;
                for (const auto& in_pair : in) {
                    const auto& in_ans = in_pair.second;
                    const auto& in_no = in_ans.no;
                    const NodeRef dst = in_pair.first;
                    const z3::expr addr = Node::same_addr(lookup(src), lookup(dst));
                    const z3::expr out_yes = in_no && src_node.arch && addr;
                    const z3::expr out_no = in_no && z3::implies(src_node.arch, !addr);
                    out.emplace(dst, Answer {out_yes, out_no});
                }
                return out;
            } else if (src_node.is_read()) {
                Value out = in;
                assert(out.find(src) == out.end());
                out.emplace(src, default_answer);
                return out;
            } else {
                return in;
            }
        };
        const auto meet = [&] (const Value& a, const Value& b) -> Value {
            Value res = a;
            for (const auto& pair : b) {
                const auto it = res.emplace(pair.first, default_answer).first;
                z3::expr& acc = it->second.no;
                acc = acc && pair.second.no;
            }
            return res;
        };
        
        Analysis(*this, {}, Analysis::REV, res, transfer, meet);
        
        progress = Progress(reads.size());
        for (NodeRef read : reads) {
            for (NodeRef write : pred_writes.at(read)) {
                const auto& out_write = res.out.at(write);
                const z3::expr exists_expr = lookup(read).arch && out_write.at(read).yes;
                const z3::expr exists_var = context.make_bool("rf");
                // const z3::expr& exists = res.out.at(write).at(read).yes;
                Edge edge {Edge::RF, exists_var};
                edge.constraints(exists_var == exists_expr, "rf-exists");
                add_unidir_edge(write, read, edge);
                ++nedges;
            }
            ++progress;
        }
        progress.done();
    }
    logv(3) << __FUNCTION__ << ": added " << nedges << " edges\n";
    logv(3) << __FUNCTION__ << ": skipped " << skipped << " edges\n";
    
    // same address
    logv(3) << __FUNCTION__ << ": adding constraint 'same address'\n";
    for_each_edge(Edge::RF, [&] (const NodeRef src, const NodeRef dst, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(src).same_addr(lookup(dst))), "rf-same-addr");
    });
    
    // execution condition
    logv(3) << __FUNCTION__ << ": adding constraint 'execution condition'\n";
    for_each_edge(Edge::RF, [&] (const NodeRef src, const NodeRef dst, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(src).arch && lookup(dst).arch), "rf-exec");
    });
    
    // most recent write
    /* This is piss-slow.
     * The underlying issue is that it's O(n^3), I think, since we're looking at O(n) preceding writes for each of the O(n^2) rf edges.
     */
    logv(3) << __FUNCTION__ << ": adding constraint 'most recent write'\n";
    Stopwatch s;
    progress = Progress(nedges);
    if constexpr (!construct_directly) {
        for_each_edge(Edge::RF, [&] (const NodeRef write, const NodeRef read, Edge& edge) {
            const auto& sourced_writes = pred_writes.at(read);
            
            s.start();
            const auto f = util::all_of(sourced_writes.begin(), sourced_writes.end(), [&] (const NodeRef other_write) -> z3::expr {
                return !edge_exists(write, other_write, Edge::CO);
            }, context.TRUE);
            s.stop();
            edge.constraints(z3::implies(edge.exists, f), "rf-recent-write");
            if (!edge.possible()) {
                std::cerr << "IMPOSSIBLE EDGE\n";
            }
            ++progress;
        });
    }
    progress.done();
    std::cerr << s << "\n";
    
    // required
    logv(3) << __FUNCTION__ << ": adding constraint 'required'\n";
    for (const NodeRef read : reads) {
        const auto rfs = get_edges(Direction::IN, read, Edge::RF);
        const auto f = util::any_of(rfs.begin(), rfs.end(), [] (const auto& ep) { return ep->exists; }, context.FALSE);
        Node& read_node = lookup(read);
        read_node.constraints(z3::implies(read_node.arch, f), "rf-required");
    }
}

void AEG::construct_co(const NodeRefVec& reads, const NodeRefVec& writes,const ClosureMap& pred_reads, const ClosureMap& pred_writes) {}

void AEG::construct_fr(const NodeRefVec& reads, const NodeRefVec& writes,
                       const ClosureMap& pred_reads, const ClosureMap& pred_writes) {
    Count count;
    for (const NodeRef write : writes) {
        const Node& write_node = lookup(write);
        for (const NodeRef read : pred_reads.at(write)) {
            const Node& read_node = lookup(read);
            const z3::expr f = write_node.arch && read_node.arch && write_node.same_addr(read_node);
            add_unidir_edge(read, write, Edge(Edge::FR, f));
            ++count;
        }
    }
    count.done();
}

void AEG::construct_com() {
    assert(po.nodes.size() == nodes.size());
    
    // get reads and writes
    NodeRefVec reads, writes;
    get_reads(std::back_inserter(reads));
    get_writes(std::back_inserter(writes));
}


void AEG::construct_rfx(const NodeRefSet& xreads, const NodeRefSet& xwrites) {
    Progress progress;
    
    // add edges
    unsigned nedges = 0;
    logv(3) << __FUNCTION__ << ": adding edges\n";
    progress = Progress(xreads.size() * xwrites.size());
    for (const NodeRef xread : xreads) {
        for (const NodeRef xwrite : xwrites) {
            if (xread != xwrite) {
                add_optional_edge(xwrite, xread, UHBEdge(Edge::RFX, context.TRUE));
                ++nedges;
            }
            ++progress;
        }
    }
    progress.done();
    logv(3) << __FUNCTION__ << ": added " << nedges << " edges\n";
    
    // same xstate
    logv(3) << __FUNCTION__ << ": adding constraint 'same xstate'\n";
    progress = Progress(nedges);
    for_each_edge(Edge::RFX, [&] (const NodeRef xw, const NodeRef xr, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(xw).same_xstate(lookup(xr))), "rfx-same-xstate");
        ++progress;
    });
    progress.done();
    
    // execution condition
    logv(3) << __FUNCTION__ << ": adding constraint 'execution condition'\n";
    progress = Progress(nedges);
    for_each_edge(Edge::RFX, [&] (const NodeRef xw, const NodeRef xr, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(xw).get_exec() && lookup(xr).get_exec()), "rfx-exec-cond");
        ++progress;
    });
    progress.done();
    
    // required
    logv(3) << __FUNCTION__ << ": adding constraint 'required'\n";
    progress = Progress(xreads.size());
    for (const NodeRef xread : xreads) {
        Node& xread_node = lookup(xread);
        const auto rfxs = get_edges(Direction::IN, xread, Edge::RFX);
        const auto f = util::one_of(rfxs.begin(), rfxs.end(), [] (const Edge *ep) -> z3::expr {
            return ep->exists;
        }, context.TRUE, context.FALSE);
        xread_node.constraints(z3::implies(xread_node.get_exec(), f), "rfx-required");
        ++progress;
    }
    progress.done();
}

void AEG::construct_cox(const NodeRefSet& xreads, const NodeRefSet& xwrites) {
    Progress progress;
    
    // add edges
    unsigned nedges = 0;
    logv(3) << __FUNCTION__ << ": adding edges\n";
    progress = Progress(xwrites.size() * (xwrites.size() - 1) / 2);
    for (auto it1 = xwrites.begin(); it1 != xwrites.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != xwrites.end(); ++it2) {
            add_bidir_edge(*it1, *it2, Edge {Edge::COX, context.TRUE});
            ++progress;
            ++nedges;
        }
    }
    progress.done();
    logv(3) << __FUNCTION__ << ": added " << nedges << " edges\n";
    
    
    // same xstate
    logv(3) << __FUNCTION__ << ": adding constraint 'same xstate'\n";
    progress = Progress(nedges);
    for_each_edge(Edge::COX, [&] (const NodeRef src, const NodeRef dst, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(src).same_xstate(lookup(dst))), "cox-same-xstate");
        ++progress;
    });
    progress.done();
    
    // execution condition
    logv(3) << __FUNCTION__ << ": adding constraint 'execution condition'\n";
    progress = Progress(nedges);
    for_each_edge(Edge::COX, [&] (const NodeRef src, const NodeRef dst, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(src).get_exec() && lookup(dst).get_exec()), "cox-exec-cond");
        ++progress;
    });
    progress.done();
    
    // total order -- already taken care of by specifying bidirectional edge
    
    // acyclic
    {
        Timer timer;
        logv(3) << __FUNCTION__ << ": adding constraint 'acyclic'\n";
        this->constraints(acyclic_int([] (const Edge& edge) -> bool {
            return edge.kind == Edge::COX;
        }), "cox-acyclic");
    }
}

void AEG::construct_frx(const NodeRefSet& xreads, const NodeRefSet& xwrites) {}

void AEG::construct_comx() {
    /* Set xsread, xswrite */
    NodeRefSet xswrites;
    NodeRefSet xsreads;
    for (NodeRef i = 0; i < size(); ++i) {
        Node& node = lookup(i);
        switch (node.inst.kind) {
            case Inst::READ:
                node.xsread = context.TRUE;
                node.xswrite = context.make_bool();
                xsreads.insert(i);
                xswrites.insert(i);
                // TODO: need to constrain when READ is an xswrite.
                break;
            case Inst::WRITE:
                node.xsread = context.TRUE;
                node.xswrite = context.TRUE;
                xsreads.insert(i);
                xswrites.insert(i);
                break;
            case Inst::ENTRY:
                node.xswrite = context.TRUE;
                xswrites.insert(i);
                break;
            case Inst::EXIT:
                node.xsread = context.TRUE;
                xsreads.insert(i);
                break;
            default:
                break;
        }
    }
    
#if 0
    logv(3) << "constructing rfx...\n";
    construct_rfx(xsreads, xswrites);
    logv(3) << "constructing cox...\n";
    construct_cox(xsreads, xswrites);
    logv(3) << "constructing frx...\n";
    construct_frx(xsreads, xswrites);

    // prevent rfx, cox cycles
    for_each_edge(Edge::RFX, [&] (const NodeRef src, const NodeRef dst, Edge& edge) {
        const auto f = !(edge.exists && edge_exists(dst, src, Edge::COX));
        edge.constraints(f, "no-rfx-cox-cycle");
    });
#endif
    
    logv(3) << "constructing xsaccess order...\n";
    construct_xsaccess_order(xsreads, xswrites);
}


#if 0
void AEG::construct_trans_group() {
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        const auto& preds = po.po.rev.at(ref);
        if (preds.size() == 1) {
            const NodeRef pred = *preds.begin();
            node.trans_group = context.make_int("trans_group");
            const Node& pred_node = lookup(pred);
            
            node.trans_group = z3::ite(node.trans,
                                       z3::ite(pred_node.trans,
                                               pred_node.trans_group,
                                               context.context.int_val(node.exec_order)),
                                       context.context.int_val(-1));
        } else {
            node.trans_group = context.context.int_val(-1);
        }
    }
}
#endif

void AEG::construct_exec_order() {
    // add variable
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        node.exec_order = context.make_int("exec_order");
    }
    
    // add constraints
    for (NodeRef ref : node_range()) {
        Node& node = lookup(ref);
        const auto preds = get_nodes(Direction::IN, ref, Edge::TFO);
        for (const auto& pred_pair : preds) {
            const NodeRef pred = pred_pair.first;
            const z3::expr& cond = pred_pair.second;
            const Node& pred_node = lookup(pred);
            
            const z3::expr f = z3::implies(node.get_exec() && pred_node.get_exec() && cond, node.exec_order > pred_node.exec_order);
            node.constraints(f, "exec-order");
        }
    }
}

void AEG::construct_trans_group() {
    // TODO: need to fix
    
    // trans group min
    {
        NodeRefVec order;
        po.reverse_postorder(std::back_inserter(order));
        for (NodeRef ref : order) {
            Node& node = lookup(ref);
            NodeRef pred;
            if (can_trans(ref, pred)) {
                // can be speculated
                const Node& pred_node = lookup(pred);
                if (can_introduce_trans(pred)) {
                    // can introduce speculation, so multiplex
                    // NOTE: Again, since we're looking at only trans nodes, we don't consider those weird TFO rollback edges.
                    node.trans_group_min = context.make_int("trans_group_min");
                    const z3::expr f = z3::implies(node.trans, node.trans_group_min == z3::ite(pred_node.trans, pred_node.trans_group_min, node.exec_order));
                    node.constraints(f, "trans-group-min");
                } else {
                    // cannot introduce speculation, so just reuse predecessor minimum
                    // NOTE: The reason that this is true is nuanced -- non-CFG TFO edges can only be introduced when the src is trans and dst is arch, but here we're assuming the dst is trans, so we can ignore such non-CFG edges.
                    node.trans_group_min = pred_node.trans_group_min;
                }
            } else {
                // can't be speculated, so just set to some bogus value
                node.trans_group_min = context.context.int_val(-1);
            }
        }
    }
    
    // trans group max
    {
        NodeRefVec order;
        po.postorder(std::back_inserter(order));
        for (NodeRef ref : order) {
            Node& node = lookup(ref);
            if (can_trans(ref)) {
                // can be speculated
                node.trans_group_max = context.make_int("trans_group_max");
                
                z3::expr max = node.exec_order;
                for (NodeRef succ : po.po.fwd.at(ref)) {
                    const Node& succ_node = lookup(succ);
                    max = z3::ite(succ_node.trans, succ_node.trans_group_max, max);
                }
                
                const z3::expr f = z3::implies(node.trans, node.trans_group_max == max);
                node.constraints(f, "trans-group-max");
            } else {
                // can't be speculated, so set to some bogus value
                node.trans_group_max = context.context.int_val(-1);
            }
        }
    }
}

void AEG::construct_xsaccess_order(const NodeRefSet& xsreads, const NodeRefSet& xswrites) {
    // add variables
    
    for (NodeRef ref : xsreads) {
        Node& node = lookup(ref);
        node.xsread_order = context.make_int("xsread_order");
    }
    
    for (NodeRef ref : xswrites) {
        lookup(ref).xswrite_order = context.make_int("xswrite_order");
    }
    
    // same-node xsread occur before xswrite
    for (NodeRef ref : xsreads) {
        if (xswrites.find(ref) != xswrites.end()) {
            Node& node = lookup(ref);
            node.constraints(z3::implies(node.xsread && node.xswrite, node.xsread_order < node.xswrite_order), "xsaccess-order");
        }
    }
    
#if 0
    // xswrites are total order
    logv(3) << __FUNCTION__ << ": adding constraint 'xswrites are total order' (" << xswrites.size() * xswrites.size() << ")\n";

    for (NodeRef ref1 : xswrites) {
        Node& node1 = lookup(ref1);
        for (NodeRef ref2 : xswrites) {
            if (ref1 != ref2) {
                const Node& node2 = lookup(ref2);
                const auto f = z3::implies(node1.xswrite && node2.xswrite, node1.xswrite_order != node2.xswrite_order);
                node1.constraints(f, "xswrite-total");
            }
        }
    }
#endif
    
    // bound by entry and exit
    const auto bound_all = [&] (XSAccess kind, const auto& bounds, const auto& bag, auto cmp) {
        for (NodeRef bound : bounds) {
            const Node& bound_node = lookup(bound);
            const z3::expr& order = bound_node.get_xsaccess_order(kind);
            for (NodeRef ref : bag) {
                if (bounds.find(ref) == bounds.end()) {
                    Node& node = lookup(ref);
                    if (!node.xsread.is_false()) {
                        node.constraints(z3::implies(node.xsread, cmp(order, node.xsread_order)), "xsaccess-order-bound-r");
                    }
                    if (!node.xswrite.is_false()) {
                        node.constraints(z3::implies(node.xswrite, cmp(order, node.xswrite_order)), "xsaccess-order-bound-w");
                    }
                }
            }
        }
    };
    
    bound_all(XSWRITE, NodeRefSet {entry}, node_range(), util::less<z3::expr>());
    bound_all(XSREAD, exits, node_range(), util::greater<z3::expr>());
    
    // require that all exits have same sequence number (not absolutely necessary)
    for (auto it1 = exits.begin(), it2 = std::next(it1); it2 != exits.end(); ++it1, ++it2) {
        constraints(lookup(*it1).xsread == lookup(*it2).xsread, "xswrite-exits-eq");
    }
}

z3::expr AEG::cox_exists(NodeRef src, NodeRef dst) const {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    
    const z3::expr precond = src_node.get_exec() && dst_node.get_exec() && src_node.xswrite && dst_node.xswrite && src_node.same_xstate(dst_node);
    const z3::expr diff = src_node.xswrite_order - dst_node.xswrite_order;
    const z3::expr cond = z3::ite(diff == 0, src_node.exec_order < dst_node.exec_order, diff < 0);
    
    return precond && cond;
}

z3::expr AEG::rfx_exists(NodeRef src, NodeRef dst) const {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    
    // TODO: does xsread, xswrite => exec?
    const z3::expr precond = src_node.get_exec() && dst_node.get_exec() && src_node.xswrite && dst_node.xsread && src_node.same_xstate(dst_node) && src_node.xswrite_order < dst_node.xsread_order;
    NodeRefVec xswrites;
    get_nodes_if(std::back_inserter(xswrites), [&] (const Node& node) -> bool {
        return !node.xswrite.is_false();
    });
    
    const z3::expr cond = util::all_of(xswrites.begin(), xswrites.end(), [&] (NodeRef write) -> z3::expr {
        const Node& write_node = lookup(write);
        const z3::expr f = cox_exists(src, write) && write_node.xswrite_order < dst_node.xsread_order;
        return !f;
    }, context.TRUE);
    return precond && cond;
}

z3::expr AEG::frx_exists(NodeRef src, NodeRef dst) const {
    const Node& src_node = lookup(src);
    const Node& dst_node = lookup(dst);
    
    const z3::expr cond = src_node.get_exec() && dst_node.get_exec() && src_node.xsread && dst_node.xswrite && src_node.same_xstate(dst_node) && src_node.xsread_order < dst_node.xswrite_order;
    return cond;
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
                            add_unidir_edge(in.ref, dst, Edge {Edge::ADDR, node.get_exec() && dst_node.get_exec()});
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
                                llvm::errs() << "SEEN " << *node.inst.I << "\n";
                                break;
                        }
                    }
                }
            }
            
        }
    }
}



void AEG::construct_mem() {
    std::vector<NodeRef> order;
    po.reverse_postorder(std::back_inserter(order));
    
    const z3::expr invalid = context.context.int_val(-1);
    const z3::sort mem_sort = context.context.array_sort(context.context.int_sort(), context.context.int_sort());

    for (NodeRef ref : order) {
        Node& node = lookup(ref);
        switch (node.inst.kind) {
            case Inst::ENTRY: {
                node.mem = z3::const_array(mem_sort, node.xswrite_order);
                break;
            }
                
            default: {
                // demultiplex mem from predecessors
                std::stringstream ss;
                ss << "mem" << ref;
                node.mem = context.context.constant(ss.str().c_str(), mem_sort);
                
                for (NodeRef pred : po.po.rev.at(ref)) {
                    const Node& pred_node = lookup(pred);
                    node.constraints(z3::implies(pred_node.arch, node.mem == pred_node.mem), "mem-pred");
                }
                
                if (!node.xswrite.is_false()) {
                    const z3::expr addr = z3::ite(node.xswrite, node.get_memory_address(), invalid);
                    node.mem = z3::store(node.mem, addr, node.xswrite_order);
                }
                
                break;
            }
        }
    }
}
