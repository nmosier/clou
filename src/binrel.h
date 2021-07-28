#pragma once

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

   void add_node(const T& node) {
      fwd[node];
      rev[node];
   }
      

   void erase(const T& src, const T& dst) {
      fwd[src].erase(dst);
      rev[dst].erase(src);
   }

   struct DefaultPrinter {
      void operator()(llvm::raw_ostream& os, const T& value) const {
         os << value;
      }
   };
   
   template <typename Printer = DefaultPrinter>
   void dump_graph(llvm::raw_ostream& os, Printer printer = Printer()) const;

   template <typename Printer = DefaultPrinter>
   void dump_graph(const std::string& path, Printer printer = Printer()) const {
      std::error_code ec;
      llvm::raw_fd_ostream os {path, ec};
      if (ec) {
         llvm::errs() << ec.message() << "\n";
         std::exit(1);
      }
      dump_graph(os, printer);
   }
   
private:
};


template <typename T, typename Hash>
template <typename Printer>
void binrel<T, Hash>::dump_graph(llvm::raw_ostream& os, Printer printer) const {
   os << R"=(
digraph G {
  layout=neato;
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
      printer(os, pair.first);
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
