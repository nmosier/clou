#pragma once

#include <string>
#include <unordered_set>
#include <fstream>

struct SpeculationPrimitive;

extern std::string output_dir;
extern std::unordered_set<std::string> function_names;
extern unsigned verbose;
extern unsigned spec_depth;
extern unsigned num_jobs;
extern unsigned rob_size;
extern std::vector<std::unique_ptr<SpeculationPrimitive>> speculation_primitives;

enum class LeakageSource {
    ADDR_DST,
    TAINT_TRANS,
};
extern std::unordered_set<LeakageSource> leakage_sources;

constexpr unsigned default_num_specs = 2;
constexpr unsigned default_num_unrolls = 2;

extern bool dump_constraints;
extern bool include_expr_in_constraint_name;

constexpr bool simplify_before_checking_for_false_constraints = false;
constexpr bool simplify_before_checking_for_impossible_edges = false;
constexpr bool should_name_constraints = true;
#define USE_TAINT 0

#include "uhb.h"
extern std::unordered_set<unsigned> include_edges;

extern std::ofstream log_;
