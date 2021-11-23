#pragma once

#include <unordered_set>
#include <unordered_map>

#include <llvm/Support/raw_ostream.h>

#include "util/hash.h"
#include "util/dot.h"

/* TODO
 * Parameterize the underlying containers.
 */

template <typename T, typename Hash = std::hash<T>>
class binrel {
public:
    using Set = std::unordered_set<T, Hash>;
    using Map = std::unordered_map<T, Set, Hash>;
    Map fwd;
    Map rev;
    
    void insert(const T& src, const T& dst) {
        fwd[src].insert(dst);
        rev[dst].insert(src);
    }
    
    template <typename InputIt>
    void insert(InputIt src_begin, InputIt src_end, const T& dst) {
        for (auto src_it = src_begin; src_it != src_end; ++src_it) {
            insert(*src_it, dst);
        }
    }
    
    template <typename InputIt>
    void insert(const T& src, InputIt dst_begin, InputIt dst_end) {
        for (auto dst_it = dst_begin; dst_it != dst_end; ++dst_it) {
            insert(src, *dst_it);
        }
    }
    
    template <typename InputIt1, typename InputIt2>
    void insert(InputIt1 src_begin, InputIt1 src_end, InputIt2 dst_begin, InputIt2 dst_end) {
        for (auto src_it = src_begin; src_it != src_end; ++src_it) {
            for (auto dst_it = dst_begin; dst_it != dst_end; ++dst_it) {
                insert(*src_it, *dst_it);
            }
        }
    }
    
    void add_node(const T& node) {
        fwd[node];
        rev[node];
    }
    
    void erase(const T& src, const T& dst) {
        fwd[src].erase(dst);
        rev[dst].erase(src);
    }
    
    void erase(const T& node) {
        const auto f = [&] (Map& rel1, Map& rel2) {
            auto it = rel1.find(node);
            assert(it != rel1.end());
            for (const T& other : it->second) {
                rel2.at(other).erase(node);
            }
            it->second.clear();
            rel1.erase(it);
        };
        
        f(fwd, rev);
        f(rev, fwd);
    }
    
    template <typename Printer>
    void dump_graph(llvm::raw_ostream& os, Printer printer) const;
    
    template <typename Printer>
    void dump_graph(const std::string& path, Printer printer) const {
        std::error_code ec;
        llvm::raw_fd_ostream os {path, ec};
        if (ec) {
            llvm::errs() << ec.message() << "\n";
            std::exit(1);
        }
        dump_graph(os, printer);
    }
    
    using Group = std::vector<T>;
    struct GroupHash {
        size_t operator()(const Group& g) const {
            return Hash()(g.front());
        }
    };
    
    struct GroupPrinter {
        void operator()(llvm::raw_ostream& os, const Group& group) const {
            for (const T& val : group) {
                os << val << "\n";
            }
        }
    };
    
    using GroupRel = binrel<Group, GroupHash>;
    GroupRel group() const;
    
    template <typename OutputIt>
    void get_nodes(OutputIt out) const;
    
    bool operator==(const binrel& other) const {
        return fwd == other.fwd && rev == other.rev;
    }
    
    using Vec = std::vector<T>;
    
    template <typename OutputIt>
    OutputIt postorder(OutputIt out, const T& entry) const {
        Set done;
        Vec order;
        postorder_rec(done, order, entry);
        return std::copy(order.begin(), order.end(), out);
    }
    
    template <typename OutputIt>
    OutputIt reverse_postorder(OutputIt out, const T& entry) const {
        Set done;
        Vec order;
        postorder_rec(done, order, entry);
        return std::copy(order.rbegin(), order.rend(), out);
    }
    
private:
    Group get_group(const T& val) const;
    
    bool postorder_rec(Set& done, Vec& order, T val) const {
        if (done.find(val) != done.end()) {
           return true;
        }

        bool acc = true;
        for (T succ : fwd.at(val)) {
           acc = acc && postorder_rec(done, order, succ);
        }

        if (acc) {
           done.insert(val);
           order.push_back(val);
        }

        return acc;
    }
};


template <typename T, typename Hash>
template <typename Printer>
void binrel<T, Hash>::dump_graph(llvm::raw_ostream& os, Printer printer) const {
    os << R"=(
digraph G {
  overlap=scale;
  splines=true;
  
)=";
    
    // define nodes
    unsigned next_id = 0;
    std::unordered_map<T, std::string> nodes;
    std::array<const Map *, 2> rels = {&fwd, &rev};
    for (const Map *rel : rels) {
        for (const auto& pair : *rel) {
            const auto res = nodes.emplace(pair.first, std::string("node") + std::to_string(next_id));
            if (res.second) {
                ++next_id;
            }
        }
    }
    for (const auto& pair : nodes) {
        os << pair.second << " [label=\"";
        
        std::string s;
        llvm::raw_string_ostream ss {s};
        printer(ss, pair.first);
        dot::quote(os, s);
        os << "\"]\n";
    }
    os << "\n";
    
    // define edges
    for (const auto& pair : fwd) {
        os << nodes.at(pair.first) << " -> {";
        for (const auto& dst : pair.second) {
            os << nodes.at(dst) << " ";
        }
        os << "}\n";
    }
    
    os << "}\n";
}


template <typename T, typename Hash>
typename binrel<T, Hash>::Group binrel<T, Hash>::get_group(const T& node_) const {
    const T *node = &node_;
    Group g;
    
    /* find entry */
    std::unordered_set<const T *> seen;
    while (true) {
        const auto seen_res = seen.insert(node);
        if (!seen_res.second) {
            return Group {node_};
        }
        const auto& preds = rev.at(*node);
        if (preds.size() != 1) { break; }
        const T& pred = *preds.begin();
        if (fwd.at(pred).size() != 1) { break; }
        node = &pred;
    }
    
    /* construct BB */
    while (true) {
        g.push_back(*node);
        const auto& succs = fwd.at(*node);
        if (succs.size() != 1) { break; }
        const T& succ = *succs.begin();
        if (rev.at(succ).size() != 1) { break; }
        node = &succ;
    }
    
    return g;
}


template <typename T, typename Hash>
template <typename OutputIt>
void binrel<T, Hash>::get_nodes(OutputIt out) const {
    for (const auto& rel : std::array<const Map *, 2> {&fwd, &rev}) {
        for (const auto& pair : *rel) {
            *out++ = pair.first;
        }
    }
}

template <typename T, typename Hash>
typename binrel<T, Hash>::GroupRel binrel<T, Hash>::group() const {
    GroupRel rel;
    
    std::unordered_set<T, Hash> nodes;
    get_nodes(std::inserter(nodes, nodes.end()));
    
    for (const T& node : nodes) {
        rel.add_node(get_group(node));
    }
    
    for (const T& src : nodes) {
        const Group src_bb = get_group(src);
        for (const T& dst : fwd.at(src)) {
            const Group dst_bb = get_group(dst);
            if (src_bb != dst_bb) {
                rel.insert(src_bb, dst_bb);
            }
        }
    }
    
    return rel;
}

