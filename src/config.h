#pragma once

#include <string>
#include <unordered_set>
#include <fstream>
#include <optional>
#include <memory>

/* FORWARD DECLARATIONS */
namespace mon {
class Client;
}

struct SpeculationPrimitive;

extern std::string output_dir;
extern std::unordered_set<std::string> function_names;
extern unsigned verbose;
extern unsigned spec_depth;
extern unsigned max_parallel;
extern unsigned rob_size;
extern std::vector<std::unique_ptr<SpeculationPrimitive>> speculation_primitives;
extern std::optional<unsigned> max_transient_nodes;
extern unsigned max_traceback;
extern bool witness_executions;
extern bool partial_executions;
extern bool fast_mode;
extern bool batch_mode;
extern mon::Client client;
extern std::optional<unsigned> stb_size;
extern bool use_lookahead;
extern unsigned window_size;
extern bool profile;
extern std::size_t distinct_limit;

struct OutputCFGs {
    bool unrolled, calls, expanded;
    
    void clearall() {
        unrolled = calls = expanded = false;
    }
    
    void setall() {
        unrolled = calls = expanded = true;
    }
    
    OutputCFGs() { setall(); }
};
extern OutputCFGs output_cfgs;

struct AliasMode {
    bool transient = true; /*!< enable alias analysis on transient instructions too (default: off) */
    bool llvm_only = false; /*!< only use LLVM alias analysis; disable all additional rules */
    
    void clear_all() {
        transient = llvm_only = false;
    }
};
extern AliasMode alias_mode;

enum class LeakageClass {
    INVALID,
    SPECTRE_V1,
    SPECTRE_V4,
    SPECTRE_PSF,
    ALL,
};
extern LeakageClass leakage_class;

struct SpectreV1Mode {
    enum Mode {
        CLASSIC,
        BRANCH_PREDICATE, // TODO: rename to 'CONTROL'
    } mode = Mode::CLASSIC;
};
extern SpectreV1Mode spectre_v1_mode;

struct SpectreV4Mode {
    bool psf = false; /// whether to allow psf
};
extern SpectreV4Mode spectre_v4_mode;

struct SyntacticDependencies {
    bool addr = true;
    bool data = true;
    
    void set_all() {
        addr = true;
        data = true;
    }
    
    void clear_all() {
        addr = false;
        data = false;
    }
};
extern SyntacticDependencies respect_syntactic_dependencies;

inline bool g_psf() {
    return leakage_class == LeakageClass::SPECTRE_V4 && spectre_v4_mode.psf;
}

constexpr unsigned default_num_specs = 2;
constexpr unsigned default_num_unrolls = 2;
constexpr unsigned recursive_call_limit = 2;

extern bool dump_constraints;
extern bool include_expr_in_constraint_name;

constexpr bool simplify_before_checking_for_false_constraints = false;
constexpr bool simplify_before_checking_for_impossible_edges = false;
constexpr bool should_name_constraints = true;

extern std::unordered_set<unsigned> include_edges;

extern std::ofstream log_;


void check_config();

class SharedDatabaseListSet;
extern SharedDatabaseListSet analyzed_functions;

void open_log(const std::string& name);
void close_log();
