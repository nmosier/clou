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
LeakageClass leakage_class;
std::optional<unsigned> max_transient_nodes;
AliasMode alias_mode = {
    .transient = false,
    .lax = false,
};
SpectreV1Mode spectre_v1_mode = {
};
SpectreV4Mode spectre_v4_mode = {
    .max_traceback = 1,
};

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
--leakage-class <class>
                     leakage class (choices: "spectre-v1", "spectre-v4", "spectre-psf", "all")
--max-transient <num>
                     set maximum number of transient nodes (default: no limit)
--aa <flag>[,<flag>...]
                     set alias analysis flags. Accepted flags: "transient", "lax"
--spectre-v1 <subopts>
                     set Spectre-v1 options. Suboptions: (none)
--spectre-v4 <subopts>
                     set Spectre-v4 options. Suboptions:
    max-traceback=<uint>  maximum traceback of addr edges via rf + {addr, data} edges
    stb-size=<uint>       store buffer size
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
        if (*tok != '\0') {
            *out++ = handler(tok);
        }
    }
    return out;
}

static bool parse_bool(const std::string& s) {
    const std::unordered_set<std::string> yes = {"yes", "y", "on"};
    const std::unordered_set<std::string> no = {"no", "n", "off"};
    std::string lower;
    std::transform(s.begin(), s.end(), std::back_inserter(lower), tolower);
    if (yes.find(lower) != yes.end()) { return true; }
    if (no.find(lower) != no.end()) { return false; }
    error("invalid boolean flag '%s'", s.c_str());
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
        LEAKAGE_CLASS,
        MAX_TRANSIENT,
        AA_FLAGS,
        SPECTRE_V1,
        SPECTRE_V4,
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
        {"leakage-class", required_argument, nullptr, LEAKAGE_CLASS},
        {"max-transient", required_argument, nullptr, MAX_TRANSIENT},
        {"aa", optional_argument, nullptr, AA_FLAGS},
        {"spectre-v1", optional_argument, nullptr, SPECTRE_V1},
        {"spectre-v4", optional_argument, nullptr, SPECTRE_V4},
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
                
            case LEAKAGE_CLASS: {
                const std::string s = optarg;
                if (s == "spectre-v1") {
                    leakage_class = LeakageClass::SPECTRE_V1;
                } else if (s == "spectre-v4") {
                    leakage_class = LeakageClass::SPECTRE_V4;
                } else if (s == "spectre-psf") {
                    leakage_class = LeakageClass::SPECTRE_PSF;
                } else if (s == "all") {
                    leakage_class = LeakageClass::ALL;
                } else {
                    error("invalid leakage class '%s'", s.c_str());
                }
                break;
            }
                
            case MAX_TRANSIENT:
                max_transient_nodes = std::stoul(optarg);
                break;
                
            case AA_FLAGS: {
                alias_mode = {
                    .transient = false,
                    .lax = false,
                };
                
                std::vector<std::string> flags;
                parse_list(optarg, std::back_inserter(flags), [] (const char *s) -> std::string { return s; });
                for (const std::string& flag : flags) {
                    if (flag == "transient") {
                        alias_mode.transient = true;
                    } else if (flag == "lax") {
                        alias_mode.lax = true;
                    } else {
                        error("bad alias analysis flag '%s", suboptarg);
                    }
                }
                break;
            }
                
            case SPECTRE_V1: {
                leakage_sources.clear();

                enum Key {
                    MODE,
                    COUNT
                };
                
                const char *keylist[COUNT + 1] = {
                    [MODE] = "mode",
                    [COUNT] = nullptr
                };
                
                bool args[COUNT] = {
                    [MODE] = true,
                };
                
                char *value;
                int idx;
                while ((idx = getsubopt(&optarg, (char **) keylist, &value)) >= 0) {
                    if (args[idx] && value == nullptr) {
                        error("spectre-v1: suboption '%s' missing value", suboptarg);
                    }
                    switch (idx) {
                        case MODE: {
                            static const std::unordered_map<std::string, SpectreV1Mode::Mode> map {
                                {"classic", SpectreV1Mode::CLASSIC},
                                {"branch-predicate", SpectreV1Mode::BRANCH_PREDICATE},
                            };
                            spectre_v1_mode.mode = map.at(value);
                            break;
                        }
                            
                        default: std::abort();
                    }
                }
                    
                // TODO: shouldn't need to configure this
                switch (spectre_v1_mode.mode) {
                    case SpectreV1Mode::CLASSIC:
                        leakage_sources = {LeakageSource::ADDR_DST};
                        break;
                    case SpectreV1Mode::BRANCH_PREDICATE:
                        leakage_sources = {LeakageSource::CTRL_DST};
                        break;
                    default: std::abort();
                }
                
                leakage_class = LeakageClass::SPECTRE_V1;
                
                break;
            }
                
                
            case SPECTRE_V4: {
                leakage_sources.clear();
                leakage_class = LeakageClass::SPECTRE_V4;
                
                enum Key {
                    MAX_TRACEBACK,
                    STB_SIZE,
                    COUNT
                };

                const char *keylist[COUNT + 1] = {
                    [MAX_TRACEBACK] = "max-traceback",
                    [STB_SIZE] = "stb-size",
                    [COUNT] = nullptr
                };
                
                bool args[COUNT] = {
                    [MAX_TRACEBACK] = true,
                    [STB_SIZE] = true,
                };
                
                char *value;
                int idx;
                while ((idx = getsubopt(&optarg, (char **) keylist, &value)) >= 0) {
                    if (args[idx] && value == nullptr) {
                        error("spectre-v4: suboption '%s' missing value", suboptarg);
                    }
                    switch (idx) {
                        case MAX_TRACEBACK:
                            spectre_v4_mode.max_traceback = std::stoul(value);
                            break;
                            
                        case STB_SIZE:
                            spectre_v4_mode.stb_size = std::stoul(value);
                            break;

                        default: std::abort();
                    }
                }
                if (suboptarg != nullptr) {
                    error("spectre-v4: invalid suboption '%s'", suboptarg);
                }
                
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
