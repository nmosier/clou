#pragma once

#include <string>
#include <cassert>
#include <unordered_set>
#include <sstream>

#include <llvm/IR/Instruction.h>
#include <llvm/Analysis/AliasAnalysis.h>
#include <z3++.h>

#include "z3-util.h"
#include "aeg-po2.h"
#include "graph.h"
#include "inst.h"
#include "util.h"

enum XSAccess {
   XSREAD, XSWRITE
};

class UHBContext {
public:
   UHBContext(): context(), TRUE(context.bool_val(true)), FALSE(context.bool_val(false)) {}
   
   z3::context context;

   z3::expr make_bool(const std::string& s = "") {
       return context.bool_const(get_name(s).c_str());
   }
    
   z3::expr make_int(const std::string& s = "") {
       return context.int_const(get_name(s).c_str());
   }

   const z3::expr TRUE;
   const z3::expr FALSE;
   
   z3::expr to_expr(bool b) const {
      return b ? TRUE : FALSE;
   }

private:
    std::unordered_map<std::string, unsigned> next;
    
    std::string get_name(const std::string& s) {
        unsigned& idx = next[s];
        return s + std::to_string(idx++);
    }
};

extern unsigned constraint_counter;

struct UHBConstraints {
    std::vector<std::pair<z3::expr, std::string>> exprs;

   UHBConstraints() {}
    explicit UHBConstraints(const z3::expr& expr, const std::string& name): exprs({{expr, name}}) {}

   void add_to(z3::solver& solver) const {
      for (const auto& p : exprs) {
         std::stringstream ss;
         ss << p.first << ":" << p.second << ":" << constraint_counter++;
         try {
            solver.add(p.first, ss.str().c_str());
         } catch (const z3::exception& e) {
            logv(0) << e.what() << "\n";
            std::abort();
         }
      }
   }
   void operator()(const z3::expr& clause, const std::string& name = "") {
      if (clause.simplify().is_false()) {
         throw std::logic_error("adding constraint 'false'");
      }
      exprs.emplace_back(clause, name);
   }
   void simplify() {
      std::for_each(exprs.begin(), exprs.end(), [] (auto& p) {
          z3::expr& e = p.first;
         e = e.simplify();
      });
   }
};

std::ostream& operator<<(std::ostream& os, const UHBConstraints& c);

struct UHBAddress {
   z3::expr arch;
   z3::expr trans;
   UHBAddress(UHBContext& ctx): arch(ctx.make_int("arch")), trans(ctx.make_int("trans")) {}
};

inline std::ostream& operator<<(std::ostream& os, const UHBAddress& x) {
   return os << "(arch)" << x.arch << " (trans)" << x.trans;
}

struct UHBNode {
   Inst inst;
   z3::expr arch;  // program order variable
   z3::expr trans; // transient fetch order variable
   z3::expr trans_depth; // transient depth
   std::optional<UHBAddress> addr_def;
   std::vector<std::pair<const llvm::Value *, UHBAddress>> addr_refs;
   z3::expr xsread;
   z3::expr xswrite;
   UHBConstraints constraints;

   z3::expr get_addr_def() const {
      // TODO: Do we need to add an extra ite to qualify based on tfo bool too?
      return z3::ite(arch, addr_def->arch, addr_def->trans);
   }

   z3::expr get_addr_ref(std::size_t idx) const {
      const UHBAddress& addr = addr_refs.at(idx).second;
      return z3::ite(arch, addr.arch, addr.trans);
   }

   z3::expr get_exec() const {
      return arch || trans;
   }
   
   void simplify() {
      arch = arch.simplify();
      trans = trans.simplify();
      trans_depth = trans_depth.simplify();
      constraints.simplify();
   }

   z3::expr same_xstate(const UHBNode& other) const {
      return get_addr_ref(0) == other.get_addr_ref(0);
   }

   z3::expr get_xsaccess(XSAccess kind) const {
      switch (kind) {
      case XSAccess::XSREAD: return xsread;
      case XSAccess::XSWRITE: return xswrite;
      default: std::abort();
      }
   }

   UHBNode(const Inst& inst, UHBContext& c);
};

struct UHBEdge {
#define UHBEDGE_KIND_X(X)                       \
   X(FORK)                                      \
   X(PO)                                        \
   X(TFO)                                       \
   X(RF)                                        \
   X(CO)                                        \
   X(FR)                                        \
   X(RFX)                                       \
   X(COX)                                       \
   X(FRX)                                       

#define UHBEDGE_KIND_E(name) name,
   enum Kind {
      UHBEDGE_KIND_X(UHBEDGE_KIND_E)
   };
#undef UHBEDGE_KIND_E

   Kind kind;
   z3::expr exists;
   UHBConstraints constraints;

   static const char *kind_tostr(Kind kind);
   const char *kind_tostr() const { return kind_tostr(kind); }
   static Kind kind_fromstr(const std::string& s);

   UHBEdge(Kind kind, UHBContext& ctx, const std::string& name = ""): // TODO: add name
      kind(kind), exists(ctx.make_bool(name)) {}

   UHBEdge(Kind kind, const z3::expr& exists): kind(kind), exists(exists) {}

   struct Hash {
      size_t operator()(const UHBEdge& x) const { return std::hash<Kind>()(x.kind); }
   };

   bool operator==(const UHBEdge& other) const { return kind == other.kind; }
   void simplify() { constraints.simplify(); }
};

std::ostream& operator<<(std::ostream& os, const UHBEdge& e);
inline std::ostream& operator<<(std::ostream& os, UHBEdge::Kind kind) {
   return os << UHBEdge::kind_tostr(kind);
}

class AEG {
public:
   using Node = UHBNode;
   using NodeRef = std::size_t;
   using Edge = UHBEdge;
   using graph_type = Graph<NodeRef, Edge, std::hash<NodeRef>, Edge::Hash>;

   NodeRef entry;
   NodeRef exit;

   graph_type graph;

   void construct(unsigned spec_depth, llvm::AliasAnalysis& AA);

   const Node& lookup(NodeRef ref) const { return nodes.at(static_cast<unsigned>(ref)); }
   Node& lookup(NodeRef ref) { return nodes.at(static_cast<unsigned>(ref)); }

   explicit AEG(const AEGPO2& po): po(po), context(), constraints() {}

   void dump_graph(llvm::raw_ostream& os) const;
   void dump_graph(const std::string& path) const;

   void simplify();

   void test();

   std::size_t size() const { return nodes.size(); }

   using Path = std::vector<NodeRef>;

private:
   const AEGPO2& po;
   UHBContext context;
   UHBConstraints constraints;
   std::vector<Node> nodes;
    
    unsigned num_specs() const { return po.num_specs; }

   void construct_exec();
   void construct_arch();
   void construct_trans();
   void construct_po();
   void construct_tfo();
   void construct_nodes_addr_defs();
   void construct_nodes_addr_refs();
   void construct_aliases(llvm::AliasAnalysis& AA);
   void construct_com();
   void construct_comx();

   struct CondNode {
      NodeRef ref;
      z3::expr cond;
   }; 
   template <typename OutputIt>
   void find_sourced_memops(Inst::Kind kind, NodeRef org, OutputIt out) const;
   template <typename OutputIt>
   void find_preceding_memops(Inst::Kind kind, NodeRef write, OutputIt out) const;

   using OptionalNodeExprMap = std::unordered_map<NodeRef, std::optional<z3::expr>>;
   
   template <typename OutputIt>
   void find_sourced_xsaccesses(XSAccess kind, NodeRef org, OutputIt out) const;
   std::optional<z3::expr> find_sourced_xsaccesses_po(XSAccess kind, NodeRef org, NodeRef ref,
                                                      OptionalNodeExprMap& yesses,
                                                      OptionalNodeExprMap& nos) const;
   std::optional<z3::expr> find_sourced_xsaccesses_tfo(XSAccess kind, NodeRef org, NodeRef ref,
                                                       unsigned spec_depth,
                                                       OptionalNodeExprMap& nos_po,
                                                       OptionalNodeExprMap& yesses_tfo,
                                                       OptionalNodeExprMap& nos_tfo) const;
   
   

   using NodeRange = util::RangeContainer<NodeRef>;
   NodeRange node_range() const {
      return NodeRange {NodeRef {entry}, NodeRef {static_cast<unsigned>(nodes.size())}};
   }


   template <typename AEG_, typename Node_>
   class NodeIterator_Base {
   public:
      NodeIterator_Base() {}
      NodeIterator_Base(AEG_ *aeg, NodeRef ref): aeg(aeg), ref(ref) {}

      struct value_type {
         const NodeRef ref;
         Node_& node;
      };

      value_type operator*() const {
         return value_type {ref, aeg->lookup(ref)};
      }

      NodeIterator_Base& operator++() {
         ++ref;
         return *this;
      }

      NodeIterator_Base& operator++(int) {
         return ++*this;
      }

      bool operator==(const NodeIterator_Base& other) const {
         if (aeg != other.aeg) {
            throw std::logic_error("comparing node iterators from two different AEGs");
         }
         return ref == other.ref;
      }

      bool operator!=(const NodeIterator_Base& other) const {
         return !(*this == other);
      }

   private:
      AEG_ * const aeg = nullptr;
      NodeRef ref;
   };
   using NodeIterator = NodeIterator_Base<AEG, Node>;
   using ConstNodeIterator = NodeIterator_Base<const AEG, const Node>;

   auto node_range2() {
      return llvm::iterator_range<NodeIterator> {
         NodeIterator{this, 0UL},
         NodeIterator{this, size()}
      };
   }

   auto node_range2() const {
      return llvm::iterator_range<ConstNodeIterator> {
         ConstNodeIterator{this, 0UL},
         ConstNodeIterator{this, size()}
      };
   }

   void find_upstream_def(NodeRef node, const llvm::Value *addr_ref,
                          std::unordered_set<NodeRef>& out) const;

   NodeRef add_node(const Node& node) {
      const NodeRef ref = size();
      nodes.push_back(node);
      graph.add_node(ref);
      return ref;
   }

   template <typename OutputIt>
   void find_comx_window(NodeRef ref, unsigned distance, unsigned spec_depth, OutputIt out) const;

    void add_unidir_edge(NodeRef src, NodeRef dst, const UHBEdge& e) {
      graph.insert(src, dst, e);
   }

   void add_bidir_edge(NodeRef a, NodeRef b, const UHBEdge& e);
   void add_optional_edge(NodeRef src, NodeRef dst, const UHBEdge& e);

   template <typename OutputIt>
   OutputIt get_edges(Direction dir, NodeRef ref, OutputIt out, Edge::Kind kind) {
      const auto& map = graph(dir);
      for (const auto& p : map.at(ref)) {
         out = std::copy_if(p.second.begin(), p.second.end(), out, [kind] (const auto& e) {
            return e->kind == kind;
         });
      }
      return out;
   }

   std::vector<std::shared_ptr<Edge>> get_edges(Direction dir, NodeRef ref, UHBEdge::Kind kind) {
      std::vector<std::shared_ptr<Edge>> es;
      get_edges(dir, ref, std::back_inserter(es), kind);
      return es;
   }

   void output_execution(std::ostream& os, const z3::model& model) const;
   void output_execution(const std::string& path, const z3::model& model) const;

   z3::expr check_no_intervening_writes(NodeRef src, NodeRef dst) const;
   // z3::expr check_no_intervening_writes_rec(NodeRef src, NodeRef

   bool is_ancestor(NodeRef parent, NodeRef child) const;
   bool is_ancestor_a(NodeRef, NodeRef) const;
   bool is_ancestor_b(NodeRef, NodeRef) const;
    
    template <typename Function>
    void for_each_edge(Function f) {
        graph.for_each_edge(f);
    }
    
    template <typename Function>
    void for_each_edge(Function f) const {
        graph.for_each_edge(f);
    }
    
    template <typename Function>
    void for_each_edge(Edge::Kind kind, Function f) {
        graph.for_each_edge([&] (NodeRef src, NodeRef dst, Edge& edge) {
            if (edge.kind == kind) {
                f(src, dst, edge);
            }
        });
    }
    
    template <typename Function>
    void for_each_edge(Edge::Kind kind, Function f) const {
        graph.for_each_edge([&] (NodeRef src, NodeRef dst, const Edge& edge) {
            if (edge.kind == kind) {
                f(src, dst, edge);
            }
        });
    }

};


