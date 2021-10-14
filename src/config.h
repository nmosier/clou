#pragma once

#include <string>
#include <unordered_set>
#include <fstream>
#include <optional>

struct SpeculationPrimitive;

extern std::string output_dir;
extern std::unordered_set<std::string> function_names;
extern unsigned verbose;
extern unsigned spec_depth;
extern unsigned num_jobs;
extern unsigned rob_size;
extern std::vector<std::unique_ptr<SpeculationPrimitive>> speculation_primitives;
extern std::optional<unsigned> max_transient_nodes;

struct AliasMode {
    bool transient; /*!< enable alias analysis on transient instructions too (default: off) */
    bool lax; /*!< relax the alias analysis, converting llvm::MayAlias -> llvm::NoAlias. This is less complete but produces more intuitive results. */
};
extern AliasMode alias_mode;

enum class LeakageSource {
    ADDR_DST,
    CTRL_DST,
};
extern std::unordered_set<LeakageSource> leakage_sources;

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
        BRANCH_PREDICATE,
    } mode;
};
extern SpectreV1Mode spectre_v1_mode;

struct SpectreV4Mode {
    unsigned max_traceback; /// max number of instructions
    unsigned stb_size; /// store buffer size
};
extern SpectreV4Mode spectre_v4_mode;

constexpr unsigned default_num_specs = 2;
constexpr unsigned default_num_unrolls = 2;

extern bool dump_constraints;
extern bool include_expr_in_constraint_name;

constexpr bool simplify_before_checking_for_false_constraints = false;
constexpr bool simplify_before_checking_for_impossible_edges = false;
constexpr bool should_name_constraints = true;

#include "uhb.h"
extern std::unordered_set<unsigned> include_edges;

extern std::ofstream log_;
