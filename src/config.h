#pragma once

#include <string>
#include <unordered_set>

extern std::string output_dir;
extern std::unordered_set<std::string> function_names;
extern unsigned verbose;
extern unsigned spec_depth;

constexpr unsigned default_num_specs = 2;
constexpr unsigned default_num_unrolls = 2;

extern bool dump_constraints;
extern bool include_expr_in_constraint_name;

constexpr bool simplify_before_checking_for_false_constraints = false;
constexpr bool simplify_before_checking_for_impossible_edges = false;

#include "uhb.h"
extern std::unordered_set<unsigned> include_edges;
