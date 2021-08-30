#include "aeg.h"

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
    const auto exit_it = std::find_if(nodes.begin(), nodes.end(), [] (const Node& node) -> bool {
        return node.inst.kind == Inst::EXIT;
    });
    exit = exit_it - nodes.begin();
    
    for (NodeRef ref : node_range()) {
        graph.add_node(ref);
    }
    
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
#if 0
    logv(2) << "Constructing comx\n";
    construct_comx();
#endif
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
    
    for (NodeRef ref = 0; ref < size(); ++ref) {
        const AEGPO::Node& po_node = po.lookup(ref);
        Node& node = lookup(ref);
        for (const llvm::Value *V : node.inst.addr_refs) {
            const auto defs_it = po_node.refs.find(V);
            std::optional<UHBAddress> e;
            // z3::expr e {context.context};
            if (defs_it == po_node.refs.end()) {
                const llvm::Argument *A = llvm::cast<llvm::Argument>(V);
                auto main_args_it = main_args.find(A);
                if (main_args_it == main_args.end()) {
                    main_args_it = main_args.emplace(A, UHBAddress {context}).first;
                }
                e = main_args_it->second;
            } else {
                const AEGPO::NodeRefSet& defs = defs_it->second;
                
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
                        }, context.FALSE));
                        node.constraints(util::any_of<z3::expr>(defs.begin(), defs.end(),
                                                                [&] (NodeRef def) {
                            return lookup_def(def).trans == e->trans;
                        }, context.FALSE));
                    }
                }
            }
            node.addr_refs.emplace_back(V, *e);
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
    entry_node.constraints(entry_node.arch);
    // TODO: replace with "true", then substitute changes
}

void AEG::construct_trans() {
    // NOTE: depends on results of construct_tfo()
    
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
}

void AEG::construct_tfo() {
    // add edges
    for (NodeRef src : node_range()) {
        for (NodeRef dst : po.po.fwd.at(src)) {
            if (po.po.rev.at(dst).size() == 1) {
                add_unidir_edge(src, dst, Edge {Edge::TFO, context, "tfo"});
            }
        }
    }
    
    // constrain edges
    for (const auto v : node_range2()) {
        // at most one successor
        const auto edges = get_edges(Direction::OUT, v.ref, Edge::TFO);
        const auto f = util::lone_of(edges.begin(), edges.end(), [] (const auto& edge) -> z3::expr {
            return edge->exists;
        }, context.TRUE, context.FALSE);
        v.node.constraints(f, "tfo-OUT");
    }
    
    for_each_edge(Edge::TFO, [&] (NodeRef src, NodeRef dst, Edge& edge) {
        const auto f = lookup(src).get_exec() && lookup(dst).trans;
        edge.constraints(z3::implies(edge.exists, f), "tfo-exec");
    });
}

void AEG::construct_aliases(llvm::AliasAnalysis& AA) {
    using ID = AEGPO::ID;
    struct Info {
        ID id;
        const llvm::Value *V;
        z3::expr e;
    };
    std::vector<Info> addrs;
    std::unordered_set<std::pair<ID, const llvm::Value *>> seen;
    for (NodeRef i = 0; i < size(); ++i) {
        const Node& node = lookup(i);
        if (node.addr_def) {
            const ID& id = *po.lookup(i).id;
            const llvm::Value *V = node.inst.I;
            addrs.push_back({id, V, node.addr_def->arch});
            [[maybe_unused]] const auto res = seen.emplace(id, V);
            assert(res.second);
        }
    }
    
    // check for arguments
    for (NodeRef i = 0; i < size(); ++i) {
        const Node& node = lookup(i);
        const AEGPO::Node& po_node = po.lookup(i);
        for (const llvm::Value *V : node.inst.addr_refs) {
            if (const llvm::Argument *A = llvm::dyn_cast<llvm::Argument>(V)) {
                const ID id {po_node.id->func, {}};
                if (seen.emplace(id, V).second) {
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
    
    // add constraints
    for (auto it1 = addrs.begin(); it1 != addrs.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != addrs.end(); ++it2) {
            if (po.alias_valid(it1->id, it2->id)) {
                const auto alias_res = AA.alias(it1->V, it2->V);
                switch (alias_res) {
                    case llvm::NoAlias:
                        constraints(it1->e != it2->e);
                        break;
                    case llvm::MayAlias:
                        break;
                    case llvm::MustAlias:
                        constraints(it1->e == it2->e);
                        break;
                    default: std::abort();
                }
            }
        }
    }
}

void AEG::construct_rf(const NodeRefVec& reads, const NodeRefVec& writes,
                       const ClosureMap& pred_reads, const ClosureMap& pred_writes) {
    // DEBUG: print out pred writes
    for (const auto& dst : pred_writes) {
        std::cerr << dst.first << " {";
        for (NodeRef src : dst.second) {
            std::cerr << " " << src;
        }
        std::cerr << "}\n";
    }
    
    // add edges
    for (const NodeRef read : reads) {
        const auto& sourced_writes = pred_writes.at(read);
        for (const NodeRef write : sourced_writes) {
            add_optional_edge(write, read, Edge {Edge::RF, context.TRUE}, "rf");
        }
    }
    
    // same address
    for_each_edge(Edge::RF, [&] (const NodeRef src, const NodeRef dst, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(src).same_addr(lookup(dst))), "rf-same-addr");
    });
    
    // execution condition
    for_each_edge(Edge::RF, [&] (const NodeRef src, const NodeRef dst, Edge& edge) {
        edge.constraints(z3::implies(edge.exists, lookup(src).arch && lookup(dst).arch), "rf-exec");
    });
    
    // most recent write
    for_each_edge(Edge::RF, [&] (const NodeRef write, const NodeRef read, Edge& edge) {
        const auto& sourced_writes = pred_writes.at(read);
        const auto f = util::all_of(sourced_writes.begin(), sourced_writes.end(), [&] (const NodeRef other_write) -> z3::expr {
            if (other_write == write) {
                return context.TRUE;
            } else {
                return edge_exists(other_write, write, Edge::CO);
            }
        }, context.TRUE);
        edge.constraints(z3::implies(edge.exists, f), "rf-recent-write");
    });
    
    // required
    for (const NodeRef read : reads) {
        const auto rfs = get_edges(Direction::IN, read, Edge::RF);
        const auto f = util::any_of(rfs.begin(), rfs.end(), [] (const auto& ep) { return ep->exists; }, context.FALSE);
        lookup(read).constraints(f, "rf-required");
    }
}

void AEG::construct_co(const NodeRefVec& reads, const NodeRefVec& writes,
                       const ClosureMap& pred_reads, const ClosureMap& pred_writes) {
    for (const NodeRef write : writes) {
        const Node& write_node = lookup(write);
        for (const NodeRef pred_write : pred_writes.at(write)) {
            const Node& pred_write_node = lookup(pred_write);
            const z3::expr f = write_node.arch && pred_write_node.arch && write_node.same_addr(pred_write_node);
            add_unidir_edge(pred_write, write, Edge {Edge::CO, f});
        }
    }
}

void AEG::construct_fr(const NodeRefVec& reads, const NodeRefVec& writes,
                       const ClosureMap& pred_reads, const ClosureMap& pred_writes) {
    for (const NodeRef write : writes) {
        const Node& write_node = lookup(write);
        for (const NodeRef read : pred_reads.at(write)) {
            const Node& read_node = lookup(read);
            const z3::expr f = write_node.arch && read_node.arch && write_node.same_addr(read_node);
            add_unidir_edge(read, write, Edge(Edge::FR, f));
        }
    }
}

void AEG::construct_com() {
    assert(po.nodes.size() == nodes.size());
    
    // get reads and writes
    std::vector<NodeRef> reads;
    std::vector<NodeRef> writes;
    for (NodeRef ref : node_range()) {
        switch (lookup(ref).inst.kind) {
            case Inst::READ:
                reads.push_back(ref);
                break;
            case Inst::WRITE:
                writes.push_back(ref);
                break;
            default: break;
        }
    }
    
    // get predecessor writes
    const ClosureMap pred_writes = find_predecessors([&] (const NodeRef ref) {
        switch (lookup(ref).inst.kind) {
            case Inst::WRITE:
            case Inst::ENTRY:
                return true;
            default:
                return false;
        }
    });
    
    // get predecessor reads
    const ClosureMap pred_reads = find_predecessors([&] (const NodeRef ref) {
        switch (lookup(ref).inst.kind) {
            case Inst::READ:
            case Inst::EXIT:
                return true;
            default:
                return false;
        }
    });
    
    construct_co(reads, writes, pred_reads, pred_writes);
    construct_rf(reads, writes, pred_reads, pred_writes);
    construct_fr(reads, writes, pred_reads, pred_writes);
}

void AEG::construct_comx() {
    /* Set xsread, xswrite */
    std::unordered_set<NodeRef> xswrites;
    std::unordered_set<NodeRef> xsreads;
    for (NodeRef i = 0; i < size(); ++i) {
        Node& node = lookup(i);
        switch (node.inst.kind) {
            case Inst::READ:
                node.xsread = context.TRUE;
                node.xswrite = context.make_bool();
                xsreads.insert(i);
                xswrites.insert(i);
                // TODO: need to constrain when this happens
                break;
            case Inst::WRITE:
                node.xsread = context.TRUE;
                node.xswrite = context.TRUE;
                xsreads.insert(i);
                xswrites.insert(i);
                break;
            default:
                break;
        }
    }
    
    
    /* add rfx */
    /* We could just blindly add all possible combinations and then later build on this to reduce
     * the number of possible cycles.
     */
    
    /* Or, alternatively, do it in a more intelligent way: create all possible edges, attach
     * a boolean to them, and then enforce that there is exactly one incoming node with this
     * edge.
     *
     * Basically, you construct the skeleton manually, and then use FOL/Alloy-like operators to
     * put constraints on these.
     *
     * For example, rfx. We construct an overapproximation of the rfx relation -- add edges from
     * a read to all writes of the same xstate.
     * Then, apply the FOL invariant that XSReads have exactly one incoming rfx edge.
     * fol::FORALL XSRead r | fol::ONE XSWrite w (w - rfx -> r)
     */
    
#if 1
    for (NodeRef xsread : xsreads) {
        const Node& xsr = lookup(xsread);
        for (NodeRef xswrite : xswrites) {
            const Node& xsw = lookup(xswrite);
            const z3::expr path = xsr.get_exec() && xsw.get_exec();
            const z3::expr is_xstate = xsr.xsread && xsw.xswrite;
            const z3::expr same_xstate = xsr.same_xstate(xsw);
            const z3::expr exists = path && is_xstate && same_xstate;
            add_optional_edge(xswrite, xsread, UHBEdge {UHBEdge::RFX, exists});
        }
        
        // add special rfx edges with ENTRY
        add_optional_edge(entry, xsread, UHBEdge {UHBEdge::RFX, xsr.get_exec() && xsr.xsread});
    }
    
    /* rfx constraint:
     * for any edge from po to tfo, require that the po node must be an ancestor of the tfo node.
     */
#if 1
    graph.for_each_edge([&] (NodeRef write, NodeRef read, Edge& e) {
        if (e.kind == Edge::RFX) {
            const Node& nw = lookup(write);
            const Node& nr = lookup(read);
            const bool is_anc = is_ancestor(write, read);
            e.constraints(z3::implies(e.exists,
                                      z3::implies(nw.arch && nr.trans, context.to_expr(is_anc))));
        }
    });
#endif
    
#else
    
    for (NodeRef read : xsreads) {
        const Node& read_node = lookup(read);
        std::vector<CondNode> writes;
        find_sourced_xsaccesses(XSAccess::XSWRITE, read, std::back_inserter(writes));
        
        std::cerr << "rfx: " << read << " ->";
        
        for (const CondNode& write : writes) {
            std::cerr << " " << write.ref;
            
            add_optional_edge(write.ref, read, UHBEdge {
                UHBEdge::RFX, read_node.get_exec() && read_node.xsread && write.cond
            });
        }
        std::cerr << "\n";
        
#if 0
        // add special rfx edges with ENTRY
        add_optional_edge(entry, read, UHBEdge {
            UHBEdge::RFX,
            read_node.get_exec() && read_node.xsread
        });
#endif
    }
#endif
    
    /* Constrain rfx edges */
#if 1
    for (NodeRef xsread : xsreads) {
        const auto es = get_edges(Direction::IN, xsread, UHBEdge::RFX);
        const z3::expr constr = util::one_of(es.begin(), es.end(), [] (const auto & e) {
            return e->exists;
        }, context.TRUE, context.FALSE);
        lookup(xsread).constraints(constr);
    }
#endif
    
    /* Adding cox is easy -- just predicate edges on po/tfo of each node and whether they access
     * the same xstate.
     * How to select which nodes to consider?
     * Considering all will cause an edge blowup.
     */
    
    /* add cox */
    for (auto it1 = xswrites.begin(); it1 != xswrites.end(); ++it1) {
        const Node& n1 = lookup(*it1);
        for (auto it2 = std::next(it1); it2 != xswrites.end(); ++it2) {
            const Node& n2 = lookup(*it2);
            const z3::expr path = n1.get_exec() && n2.get_exec();
            const z3::expr is_xstate = n1.xswrite && n2.xswrite;
            const z3::expr same_xstate = n1.same_xstate(n2);
            const z3::expr exists = path && is_xstate && same_xstate;
            add_bidir_edge(*it1, *it2, UHBEdge {UHBEdge::COX, exists});
        }
        add_unidir_edge(entry, *it1, UHBEdge {UHBEdge::COX, n1.get_exec() && n1.xswrite});
    }
    
    /* READs only perform an XSWrite if there are no previous READ/WRITEs to that address.
     * When do we need competing pairs, anyway?
     * Only with upstream instructions.
     * For now, just use a hard-coded limit.
     *
     * Also, for now don't constrain the XSWrite boolean for READs. We can deal with this later.
     * We need to somehow enforce with comx that tfo executes before po.
     *
     * For each node:
     * First consider po case:
     */
    
    
    std::vector<graph_type::Cycle> cycles;
    graph.cycles(std::back_inserter(cycles), [] (const UHBEdge& e) -> bool {
        switch (e.kind) {
            case UHBEdge::PO:
                // case UHBEdge::TFO:
            case UHBEdge::COX:
            case UHBEdge::RFX:
            case UHBEdge::FRX:
                return true;
            default:
                return false;
        }
    });
    for (const auto& cycle : cycles) {
        const auto f =
        std::transform_reduce(cycle.edges.begin(), cycle.edges.end(), context.FALSE,
                              util::logical_or<z3::expr>(),
                              [&] (const std::vector<UHBEdge>& es) -> z3::expr {
            return !std::transform_reduce(es.begin(), es.end(), context.FALSE,
                                          util::logical_or<z3::expr>(),
                                          [] (const UHBEdge& e) {
                return e.exists;
            });
            
        });
        constraints(f);
        logv(2) << util::to_string(f) << "\n";
    }
}
