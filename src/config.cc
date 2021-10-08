#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <getopt.h>
#include <vector>

#include <llvm/Support/raw_ostream.h>

#include "config.h"
#include "spec-prim.h"

/* TODO
 * [ ] Handle function names
 */

char prog[] = "lcm";
static std::vector<char *> args = {prog};

std::string output_dir;
unsigned verbose = 0;
bool dump_constraints = false;
bool include_expr_in_constraint_name = false;
std::unordered_set<std::string> function_names;
std::unordered_set<unsigned> include_edges;
unsigned spec_depth = 2;
unsigned num_jobs = 1;
unsigned rob_size = 10;
std::vector<std::unique_ptr<SpeculationPrimitive>> speculation_primitives;
std::ofstream log_;
std::unordered_set<LeakageSource> leakage_sources;


// TODO: add automated way for describing default values

static void usage(FILE *f = stderr) {
    const char *s = R"=(usage: [option...]
Options:
--help, -h           show help
--output, -o <path>  output directory
--func, -f <name>[,<name>]...
only examine given functions
--verbose, -v        verbosity++
--constraints, -c    include constraints in AEG graph output
--expr, -e           include expression string in constraint name (for debugging)
--edges, -E          include edges in execution graph output
--depth, -d <n>      speculation depth
--rob, -r <n>        reorder buffer (ROB) size
--speculation-primitives <primitive>[,<primitive>...]
                     use comma-separated speculation primitives (possibilities: "branch", "addr")
--leakage-sources <source>[,<source>...]
                     use comman-separated leakage sources (possibilities: "addr-dst", "taint-trans")
)=";
    fprintf(f, s);
}

static void initialize() {
    speculation_primitives.push_back(std::make_unique<BranchPrimitive>());
    leakage_sources = {LeakageSource::ADDR_DST};
    log_.open("log");
}

template <typename OutputIt, typename Handler>
static OutputIt parse_list(char *s, OutputIt out, Handler handler) {
    char *tok;
    while ((tok = strsep(&s, ","))) {
        *out++ = handler(tok);
    }
    return out;
}

static int parse_args() {
    if (char *line = getenv("LCM_ARGS")) {
        while (char *s = strsep(&line, " ")) {
            if (*s) {
                args.push_back(s);
            }
        }
    }
    
    char **argv = args.data();
    int argc = args.size();
    int optc;
    
    initialize();
    
    enum Option {
        SPECULATION_PRIMITIVES = 256,
        LEAKAGE_SOURCES,
    };
    
    struct option opts[] = {
        {"help", no_argument, nullptr, 'h'},
        {"verbose", no_argument, nullptr, 'v'},
        {"output", required_argument, nullptr, 'o'},
        {"constraints", no_argument, nullptr, 'c'},
        {"expr", no_argument, nullptr, 'e'},
        {"function", required_argument, nullptr, 'f'},
        {"edges", required_argument, nullptr, 'E'},
        {"depth", required_argument, nullptr, 'd'},
        {"jobs", required_argument, nullptr, 'j'},
        {"speculation-primitives", required_argument, nullptr, SPECULATION_PRIMITIVES},
        {"leakage-sources", required_argument, nullptr, LEAKAGE_SOURCES},
        {nullptr, 0, nullptr, 0}
    };
    
    while ((optc = getopt_long(argc, argv, "hvo:cef:E:d:j:", opts, nullptr)) >= 0) {
        switch (optc) {
            case 'h':
                usage(stdout);
                exit(0);
                
            case 'o':
                output_dir = optarg;
                break;
                
            case 'v':
                ++verbose;
                break;
                
            case 'c':
                dump_constraints = true;
                break;
                
            case 'e':
                include_expr_in_constraint_name = true;
                break;
                
            case 'f':
                function_names.insert(optarg);
                break;
                
            case 'E': {
                const char *token;
                while ((token = strsep(&optarg, ",")) != nullptr) {
                    include_edges.insert(UHBEdge::kind_fromstr(token));
                }
                break;
            }
                
            case 'd':
                spec_depth = std::stoul(optarg);
                break;
                
            case 'j':
                num_jobs = std::stoul(optarg);
                break;
                
            case SPECULATION_PRIMITIVES: {
                speculation_primitives.clear();
                std::cerr << "here\n";
                parse_list(optarg, std::back_inserter(speculation_primitives), [] (const std::string& s) -> std::unique_ptr<SpeculationPrimitive>{
                    if (s == "branch") {
                        return std::make_unique<BranchPrimitive>();
                    } else if (s == "addr") {
                        return std::make_unique<AddrSpecPrimitive>();
                    } else {
                        error("invalid speculation primitive '%s'", s.c_str());
                    }
                });
                break;
            }
                
            case LEAKAGE_SOURCES: {
                leakage_sources.clear();
                parse_list(optarg, std::inserter(leakage_sources, leakage_sources.end()), [] (const std::string& s) {
                    if (s == "addr-dst") {
                        return LeakageSource::ADDR_DST;
                    } else if (s == "taint-trans") {
                        return LeakageSource::CTRL_DST;
                    } else {
                        error("invalid leakage source '%s'", s.c_str());
                    }
                });
                break;
            }
                
                
            default:
                usage();
                exit(1);
        }
    }
    
    return 0;
}

static const int parse_args_force = parse_args();
