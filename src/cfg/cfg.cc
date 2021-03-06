#include <set>
#include <llvm/IR/IntrinsicInst.h>

#include "cfg/cfg.h"
#include "util/algorithm.h"
#include "util/output.h"
#include "config.h"
#include "util/functional.h"
#include "cfg/node.h"

CFG::CFG() {}

CFG::Node& CFG::lookup(NodeRef ref) {
    return nodes.at(ref);
}

const CFG::Node& CFG::lookup(NodeRef ref) const {
    return nodes.at(ref);
}

void CFG::dump_graph(const std::string& path) const {
    po.group().dump_graph(path, [&] (auto& os, const auto& group) {
        for (NodeRef ref : group) {
            os << ref << " " << lookup(ref) << "\n";
        }
    });
}

std::size_t CFG::size() const {
    return nodes.size();
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const CFG::Node& node) {
    std::visit(util::overloaded {
        [&] (Entry) { os << "<ENTRY>"; },
        [&] (Exit)  { os << "<EXIT>";  },
        [&] (const llvm::Instruction *I) { os << *I; },
        [&] (const CFG::Node::Call& call) {
            os << *call.C << " " << call.arg;
        },
    }, node());
    if (node.id) {
        using output::operator<<;
        os << " F" << node.id->func;
        os << " L" << node.id->loop;
    }
    return os;
}

void CFG::prune() {
    std::unordered_set<NodeRef> todo;
    for (NodeRef i = 0; i < nodes.size(); ++i) {
        if (exits.find(i) == exits.end()) {
            todo.insert(i);
        }
    }
    
    NodeRefSet deleted;
    while (!todo.empty()) {
        // pop off next job
        const auto it = todo.begin();
        const NodeRef ref = *it;
        todo.erase(it);
        
        if (po.fwd.at(ref).empty()) {
            // is leaf
            const auto& preds = po.rev.at(ref);
            std::copy(preds.begin(), preds.end(), std::inserter(todo, todo.end()));
            po.erase(ref);
            deleted.insert(ref);
        }
    }
    
    /* NOTE: There is a more efficient way to do this renumbering of node references;
     * however, this method preserved the order. If it becomes too slow, we can replace it.
     */
    
    /* create mapping from old ref to new refs */
    std::unordered_map<NodeRef, NodeRef> refmap;
    NodeRef next_ref = 0;
    for (NodeRef old_ref = 0; old_ref < nodes.size(); ++old_ref) {
        if (deleted.find(old_ref) == deleted.end()) {
            refmap.emplace(old_ref, next_ref);
            ++next_ref;
        }
    }
    
    /* compactify nodes */
    Rel new_po;
    for (NodeRef old_ref = 0; old_ref < nodes.size(); ++old_ref) {
        const auto it = refmap.find(old_ref);
        if (it != refmap.end()) {
            nodes[it->second] = nodes[it->first];
            new_po.add_node(it->second);
        }
    }
    nodes.resize(next_ref);
    
    /* rename mappings */
    for (const auto& pair : po.fwd) {
        const NodeRef src = pair.first;
        for (const NodeRef dst : pair.second) {
            new_po.insert(refmap.at(src), refmap.at(dst));
        }
    }
    po = std::move(new_po);
}

bool CFG::llvm_alias_valid(const ID& a, const ID& b) {
    if (a.func != b.func) {
        return false;
    }
    
    if (!util::prefixeq_bi(a.loop, b.loop)) {
        return false;
    }
    
    return true;
}

bool CFG::llvm_alias_valid(const Node& a, const Node& b) {
    if (!(a.id && b.id)) {
        return false;
    }
    return llvm_alias_valid(*a.id, *b.id);
}

void CFG::compute_postorder(NodeRefVec& order) const {
    /* ALGORITHM:
     * Maintain a min-heap  */
    
    std::vector<std::size_t> ref2n(size());
    std::set<std::pair<std::size_t, NodeRef>> heap;
    
    // initialize heap
    for (NodeRef ref = 0; ref < size(); ++ref) {
        const auto n = po.fwd.at(ref).size();
        ref2n[ref] = n;
        heap.emplace(n, ref);
    }
    
    while (!heap.empty()) {
        const NodeRef ref = heap.begin()->second;
        assert(heap.begin()->first == 0);
        assert(ref2n.at(ref) == 0);
        heap.erase(heap.begin());
        
        // add to order
        order.push_back(ref);
        
        // decrement parent references
        for (const NodeRef pred : po.rev.at(ref)) {
            auto n = ref2n.at(pred);
            assert(n > 0);
            heap.erase(std::make_pair(n, pred));
            heap.emplace(n - 1, pred);
            --ref2n.at(pred);
        }
    }
}

NodeRef CFG::add_node(const Node& node) {
    const NodeRef ref = size();
    nodes.push_back(node);
    po.add_node(ref);
    return ref;
}

std::ostream& operator<<(std::ostream& os, const CFG::ID& id) {
    os << "F" << id.func << " L{";
    for (auto it = id.loop.begin(); it != id.loop.end(); ++it) {
        if (it != id.loop.begin()) {
            os << " ";
        }
        os << *it;
    }
    os << "}";
    return os;
}

bool CFG::is_block_boundary(NodeRef ref, const Rel::Map& fwd, const Rel::Map& rev) const {
    const auto& preds = rev.at(ref);
    if (preds.size() != 1) {
        return true;
    }
    const NodeRef pred = *preds.begin();
    const auto& pred_succs = fwd.at(pred);
    if (pred_succs.size() == 1) {
        return false;
    } else {
        return true;
    }
}

bool CFG::is_block_entry(NodeRef ref) const {
    return is_block_boundary(ref, po.fwd, po.rev);
}

bool CFG::is_block_exit(NodeRef ref) const {
    return is_block_boundary(ref, po.rev, po.fwd);
}

std::optional<NodeRef> CFG::get_block_successor(NodeRef ref) const {
    if (is_block_exit(ref)) {
        return std::nullopt;
    } else {
        return *po.fwd.at(ref).begin();
    }
}

bool CFG::Node::may_read() const {
    return std::visit(util::overloaded {
        [&] (Entry) { return false; },
        [&] (Exit) { return true; },
        [&] (const llvm::Instruction *I) {
            // TODO: more cases
            if (llvm::isa<llvm::LoadInst>(I)) {
                return true;
            } else {
                return false;
            }
        },
        [&] (const Call& call) {
            assert(call.arg->getType()->isPointerTy());
            return true;
        },
    }, v);
}


bool CFG::is_ancestor(NodeRef parent, NodeRef child) const {
    NodeRefVec todo = {child};
    NodeRefSet seen;
    while (!todo.empty()) {
        const NodeRef ref = todo.back();
        todo.pop_back();
        if (!seen.insert(ref).second) { continue; }
        if (ref == parent) { return true; }
        util::copy(po.rev.at(ref), std::back_inserter(todo));
    }
    return false;
}

const llvm::Function *CFG::function() const {
    for (NodeRef ref = 0; ref < size(); ++ref) {
        const Node& node = lookup(ref);
        if (const auto *Ip = std::get_if<const llvm::Instruction *>(&node.v)) {
            const llvm::Instruction *I = *Ip;
            return I->getFunction();
        }
    }
    std::abort();
}

std::string CFG::function_name() const {
    for (NodeRef ref = 0; ref < size(); ++ref) {
        const Node& node = lookup(ref);
        if (const auto *Ip = std::get_if<const llvm::Instruction *>(&node.v)) {
            const llvm::Instruction *I = *Ip;
            return I->getFunction()->getName().str();
        }
    }
    std::abort();
}


llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const CFG::Translations::Key& key) {
    using output::operator<<;
    os << "{.id = " << key.id << ", V = " << *key.V << "}";
    return os;
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& os, const CFG::Translations::Value& value) {
    using output::operator<<;
    os << "{.id = " << value.id << ", Vs = {";
    output::container(os, value.Vs, ", ", [] (const auto *ptr) -> const llvm::Value& { return *ptr; });
    os << "}";
    return os;
}


bool CFG::may_introduce_speculation(NodeRef ref) const {
    switch (leakage_class) {
        case LeakageClass::SPECTRE_V1:
            // check if branch inst
            if (auto *Ip = std::get_if<const llvm::Instruction *>(&lookup(ref).v)) {
                return llvm::isa<llvm::BranchInst>(*Ip) && po.fwd.at(ref).size() > 1;
            } else {
                return false;
            }
            
        case LeakageClass::SPECTRE_V4:
            for (const NodeRef ref : po.fwd.at(ref)) {
                const Node& node = lookup(ref);
                bool res = false;
                if (const auto *Ip = std::get_if<const llvm::Instruction *>(&node.v)) {
                    const llvm::Instruction *I = *Ip;
                    res = I->mayReadFromMemory();
                    if (llvm::isa<llvm::DbgInfoIntrinsic>(I)) {
                    } else if (llvm::isa<llvm::CallBase>(I) || llvm::isa<llvm::LoadInst>(I)) {
                        assert(res);
                    } else if (const llvm::StoreInst *SI = llvm::dyn_cast<llvm::StoreInst>(I)) {
                        if (SI->isVolatile()) {
                            assert(res);
                            res = false; // don't handle volatile instructions for now
                        } else {
                            assert(!res);
                        }
                    } else if (llvm::isa<llvm::FenceInst>(I)) {
                        res = false;
                    } else {
                        if (res) {
                            llvm::errs() << node << "\n";
                        }
                        assert(!res);
                    }
                }
                if (res) {
                    return true;
                }
            }
            return false;
            
        default: std::abort();
    }
}


void CFG::sort() {
    NodeRefVec order;
    compute_postorder(order);
    std::reverse(order.begin(), order.end());
    std::unordered_map<NodeRef, NodeRef> map;
    for (NodeRef newref = 0; NodeRef oldref : order) {
        map.emplace(oldref, newref);
        ++newref;
    }
    
    const Rel& oldpo = this->po;
    Rel newpo;
    
    /* add nodes */
    for (const auto& [oldref, newref] : map) {
        newpo.add_node(newref);
    }
    
    /* add edges */
    for (const auto& [oldref_src, oldref_dsts] : oldpo.fwd) {
        for (const auto& oldref_dst : oldref_dsts) {
            newpo.insert(map.at(oldref_src), map.at(oldref_dst));
        }
    }
    
    this->po = newpo;
    
    auto& oldnodes = this->nodes;
    std::vector<Node> newnodes;
    for (const auto oldref : order) {
        newnodes.push_back(std::move(oldnodes.at(oldref)));
    }
    this->nodes = newnodes;
    
    /* set entry */
    entry = map.at(entry);
    
    /* set exits */
    NodeRefSet newexits;
    util::transform(exits, std::inserter(newexits, newexits.end()), [&map] (NodeRef in) -> NodeRef {
        return map.at(in);
    });
    exits = newexits;
}

boost::integer_range<NodeRef> CFG::reverse_postorder() const {
    return boost::irange<NodeRef>(0, static_cast<NodeRef>(nodes.size()));
}


#if 0
// TODO
std::optional<NodeRefVec> CFG::unique_path(NodeRef src, NodeRef dst) const {
    // NOTE: Assumes that noderefs are in reverse postorder.
    
    if (src > dst) {
        return std::nullopt;
    } else if (src == dst) {
        NodeRefVec path = {src};
        return path;
    }
    
    NodeRefVec path;
    NodeRef ref = src;
    while (true) {
        const auto& succs = po.fwd.at(ref);
        switch (succs.size()) {
            case 0:
                return std::nullopt;
                
            case 1:
                path.push_back(ref);
                ref = *succs.begin();
                break;
                
            default:
                for (const auto& succs)
        }
        
    }
}
#endif

// check if in same basic block

bool CFG::same_basic_block(NodeRef src, NodeRef dst) const {
    NodeRef ref = src;
    while (true) {
        if (ref > dst) {
            return false;
        } else if (ref == dst) {
            return true;
        }
        
        const auto& succs = po.fwd.at(ref);
        if (succs.size() != 1) {
            return false;
        }
        
        ref = *succs.begin();
    }
}


NodeRefSet CFG::prune_exec_window(const NodeRefSet& window) const {
    /* first pass:
     * - postorder
     * - each node gets common prefix */
    std::unordered_map<NodeRef, std::vector<FuncID>> map;
    
    const auto get_id = [&] (NodeRef ref) {
        const auto& node = lookup(ref);
        if (node.id) {
            return node.id->func;
        } else {
            return std::vector<FuncID>();
        }
    };
    
    for (NodeRef ref : postorder()) {
        if (!window.contains(ref)) { continue; }
        auto succs = po.fwd.at(ref);
        std::erase_if(succs, [&window] (NodeRef ref) {
            return !window.contains(ref);
        });
        auto out = std::transform_reduce(succs.begin(), succs.end(), get_id(ref), [] (const auto& f1, const auto& f2) {
            std::vector<FuncID> res;
            util::shared_prefix(f1, f2, std::back_inserter(res));
            return res;
        }, get_id);
        map.emplace(ref, std::move(out));
    }
    
    /* reverse pass */
    for (NodeRef ref : reverse_postorder()) {
        if (!window.contains(ref)) { continue; }
        
        auto preds = po.rev.at(ref);
        std::erase_if(preds, [&window] (NodeRef ref) {
            return !window.contains(ref);
        });
        auto out = std::transform_reduce(preds.begin(), preds.end(), map.at(ref), [] (const auto& f1, const auto& f2) {
            std::vector<FuncID> res;
            util::shared_prefix(f1, f2, std::back_inserter(res));
            return res;
        }, [&] (NodeRef ref) {
            return map.at(ref);
        });
        
        map.at(ref) = std::move(out);
    }
    
    /* remove nodes that don't reach main function */
    NodeRefSet newwindow;
    for (NodeRef ref : window) {
        if (map.at(ref).empty()) {
            newwindow.insert(ref);
        }
    }
    
    return newwindow;
}
