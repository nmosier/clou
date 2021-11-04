#pragma once

#include <string>
#include <unordered_set>
#include <fstream>
#include <optional>

/* FORWARD DECLARATIONS */
namespace mon {
class Client;
}

struct SpeculationPrimitive;

extern std::string output_dir;
extern std::unordered_set<std::string> function_names;
extern unsigned verbose;
extern unsigned spec_depth;
extern unsigned num_jobs;
extern unsigned rob_size;
extern std::vector<std::unique_ptr<SpeculationPrimitive>> speculation_primitives;
extern std::optional<unsigned> max_transient_nodes;
extern unsigned max_traceback;
extern bool witness_executions;
extern bool partial_executions;
extern bool fast_mode;
extern bool batch_mode;
extern std::optional<mon::Client> client;

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
    bool transient; /*!< enable alias analysis on transient instructions too (default: off) */
    bool lax; /*!< relax the alias analysis, converting llvm::MayAlias -> llvm::NoAlias. This is less complete but produces more intuitive results. */
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
    } mode;
};
extern SpectreV1Mode spectre_v1_mode;

struct SpectreV4Mode {
    bool psf; /// whether to allow psf
    unsigned stb_size; /// store buffer size
};
extern SpectreV4Mode spectre_v4_mode;

inline bool g_psf() {
    return leakage_class == LeakageClass::SPECTRE_V4 && spectre_v4_mode.psf;
}

constexpr unsigned default_num_specs = 2;
constexpr unsigned default_num_unrolls = 2;

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
