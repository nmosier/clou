#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <getopt.h>
#include <vector>
#include <csignal>

#include <llvm/Support/raw_ostream.h>

#include "config.h"
#include "util.h"
#include "uhb.h"
#include "db.h"
#include "mon/client.h"

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
unsigned max_traceback = 1;
std::ofstream log_;
LeakageClass leakage_class = LeakageClass::INVALID;
std::optional<unsigned> max_transient_nodes;
AliasMode alias_mode;
SpectreV1Mode spectre_v1_mode;
SpectreV4Mode spectre_v4_mode;
bool witness_executions = true;
bool partial_executions = false;
bool fast_mode = false;
bool batch_mode = false;
std::optional<unsigned> stb_size;
SyntacticDependencies respect_syntactic_dependencies;

OutputCFGs output_cfgs;

mon::Client client;

SharedDatabaseListSet analyzed_functions;

namespace {

void usage(FILE *f = stderr) {
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
--speculation-primitives <primitive>[,<primitive>...]
                     use comma-separated speculation primitives (possibilities: "branch", "addr")
--leakage-sources <source>[,<source>...]
                     use comman-separated leakage sources (possibilities: "addr-dst", "taint-trans")
--max-transient <num>
                     set maximum number of transient nodes (default: no limit)
--aa <flag>[,<flag>...]
                     set alias analysis flags. Accepted flags: "transient", "llvm-only"
--spectre-v1 <subopts>
                     set Spectre-v1 options. Suboptions:
    mode={classic|branch-predicate}
--spectre-v4 <subopts>
                     set Spectre-v4 options. Suboptions:
    stb-size=<uint>       store buffer size
--traceback <uint>   set max traceback via rf * (addr + data) edges.
--witnesses <bool>   enable/disable generation of witness executions (default: on)
--partial[=<bool>]   model partial executions in AEG (default: false)
--fast[=<bool>]      enable/disable fast mode (default: off)
--batch              batch mode (when all leakage output is going to one file)
--cfg[=<type>...]    output CFGs. Types: "unrolled", "calls", "expanded"
--stb <value>        store buffer size (default: "unlimited")
--respect-syntactic-deps[=<dep>...]
                     respect syntactic dependencies (options: "addr", "data"). Assign empty string to respect none.
)=";
    fprintf(f, "%s", s);
}

void initialize() {
    log_.open("log");
}

template <typename OutputIt, typename Handler>
OutputIt parse_list(char *s, OutputIt out, Handler handler) {
    char *tok;
    while ((tok = strsep(&s, ","))) {
        if (*tok != '\0') {
            *out++ = handler(tok);
        }
    }
    return out;
}

bool parse_bool(const std::string& s) {
    const std::unordered_set<std::string> yes = {"yes", "y", "on"};
    const std::unordered_set<std::string> no = {"no", "n", "off"};
    std::string lower;
    std::transform(s.begin(), s.end(), std::back_inserter(lower), tolower);
    if (yes.find(lower) != yes.end()) { return true; }
    if (no.find(lower) != no.end()) { return false; }
    error("invalid boolean flag '%s'", s.c_str());
}

bool parse_bool_opt(const char *s) {
    if (s) {
        return parse_bool(s);
    } else {
        return true;
    }
}

void initialize_post() {
    analyzed_functions = SharedDatabaseListSet(util::to_string(output_dir, "/functions.txt"));
    
    std::signal(SIGPIPE, SIG_IGN);
}

int parse_args() {
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
        MAX_TRANSIENT,
        AA_FLAGS,
        SPECTRE_V1,
        SPECTRE_V4,
        TRACEBACK,
        WITNESSES,
        PARTIAL,
        FAST,
        BATCH,
        MONITOR,
        CFG,
        STB,
        SYNTACTIC_DEPS,
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
        {"max-transient", required_argument, nullptr, MAX_TRANSIENT},
        {"aa", optional_argument, nullptr, AA_FLAGS},
        {"spectre-v1", optional_argument, nullptr, SPECTRE_V1},
        {"spectre-v4", optional_argument, nullptr, SPECTRE_V4},
        {"traceback", required_argument, nullptr, TRACEBACK},
        {"witnesses", optional_argument, nullptr, WITNESSES},
        {"partial", optional_argument, nullptr, PARTIAL},
        {"fast", optional_argument, nullptr, FAST},
        {"batch", optional_argument, nullptr, BATCH},
        {"monitor", required_argument, nullptr, MONITOR},
        {"cfg", optional_argument, nullptr, CFG},
        {"stb", required_argument, nullptr, STB},
        {"respect-syntactic-deps", optional_argument, nullptr, SYNTACTIC_DEPS},
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
                    include_edges.insert(aeg::Edge::kind_fromstr(token));
                }
                break;
            }
                
            case 'd':
                spec_depth = std::stoul(optarg);
                break;
                
            case 'j':
                num_jobs = std::stoul(optarg);
                break;
                
            case MAX_TRANSIENT:
                max_transient_nodes = std::stoul(optarg);
                break;
                
            case AA_FLAGS: {
                alias_mode.clear_all();
                
                static const std::unordered_map<std::string, bool AliasMode::*> map = {
                    {"transient", &AliasMode::transient},
                    {"llvm-only", &AliasMode::llvm_only},
                };
                char *tok;
                while ((tok = ::strsep(&optarg, ",")) != nullptr) {
                    const auto it = map.find(tok);
                    if (it == map.end()) {
                        error("bad alias analysis flag '%s'", tok);
                    } else {
                        alias_mode.*it->second = true;
                    }
                }
                break;
            }
            
            case SPECTRE_V1: {
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
                while ((idx = ::getsubopt(&optarg, (char **) keylist, &value)) >= 0) {
                    if (args[idx] && value == nullptr) {
                        error("spectre-v1: suboption missing value");
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
                
                leakage_class = LeakageClass::SPECTRE_V1;
                
                break;
            }
                
                
            case SPECTRE_V4: {
                leakage_class = LeakageClass::SPECTRE_V4;
                
                enum Key {
                    PSF,
                    STB_SIZE,
                    COUNT
                };

                const char *keylist[COUNT + 1] = {
                    [PSF] = "psf",
                    [STB_SIZE] = "stb-size",
                    [COUNT] = nullptr
                };
                
                bool args[COUNT] = {
                    [PSF] = false,
                    [STB_SIZE] = true,
                };
                
                char *value;
                int idx;
                while (optarg && (idx = getsubopt(&optarg, (char **) keylist, &value)) >= 0) {
                    if (args[idx] && value == nullptr) {
                        error("spectre-v4: suboption missing value");
                    }
                    switch (idx) {
                        case PSF:
                            spectre_v4_mode.psf = true;
                            break;
                            
                        default: std::abort();
                    }
                }
                if (optarg && *optarg) {
                    error("spectre-v4: invalid suboption");
                }
                
                break;
            }
                
                
            case TRACEBACK: {
                max_traceback = std::stoul(optarg);
                break;
            }
                
            case WITNESSES: {
                witness_executions = parse_bool_opt(optarg);
                break;
            }
                
            case PARTIAL: {
                partial_executions = parse_bool_opt(optarg);
                break;
            }
                
            case FAST: {
                fast_mode = parse_bool_opt(optarg);
                if (fast_mode) {
                    witness_executions = false;
                    partial_executions = true;
                    output_cfgs.clearall();
                    output_cfgs.expanded = true;
                }
                break;
            }
                
            case BATCH: {
                batch_mode = parse_bool_opt(optarg);
                break;
            }
                
            case MONITOR: {
                client = mon::Client(optarg);
                client.send_connect();
                break;
            }
                
            case CFG: {
                if (optarg) {
                    static const std::unordered_map<std::string, bool OutputCFGs::*> map = {
                        {"unrolled", &OutputCFGs::unrolled},
                        {"calls", &OutputCFGs::calls},
                        {"expanded", &OutputCFGs::expanded},
                    };
                    char *tok;
                    while ((tok = ::strsep(&optarg, ",")) != nullptr) {
                        const auto it = map.find(tok);
                        if (it == map.end()) {
                            error("invalid CFG '%s'", tok);
                        }
                        output_cfgs.*it->second = true;
                    }
                } else {
                    output_cfgs.setall();
                }
                break;
            }
                
            case STB: {
                if (std::string(optarg) == "unlimited") {
                    stb_size = std::nullopt;
                } else {
                    stb_size = std::stoul(optarg);
                }
                break;
            }
                
            default:
                usage();
                exit(1);
        }
    }
    
    
    // check
    // check_config();
    initialize_post();
    
    std::cerr << "llvm is multithreaded: " << llvm::llvm_is_multithreaded() << "\n";
    
    return 0;
}

const int parse_args_force = parse_args();

}

void check_config() {
    if (leakage_class == LeakageClass::INVALID) {
        throw util::resume("warning: missing leakage class option (--spectre-v1, --spectre-v4, ...)");
    }
}
