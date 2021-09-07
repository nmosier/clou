#pragma once

#include <string>

extern std::string output_dir;
extern std::vector<std::string> function_names;
extern unsigned verbose;

constexpr unsigned default_num_specs = 2;
constexpr unsigned default_num_unrolls = 2;

extern bool dump_constraints;
extern bool include_expr_in_constraint_name;
