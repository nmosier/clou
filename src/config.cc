#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <getopt.h>
#include <vector>
#include <csignal>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/stat.h>
#include <gdbm.h>

#include <llvm/Support/raw_ostream.h>

#include "config.h"
#include "db.h"
#include "mon/client.h"
#include "util/exception.h"
#include "util/output.h"
#include "aeg/edge.h"
#include "aeg/node.h"

/* TODO
 * [ ] Handle function names
 */

char prog[] = "lcm";
static std::vector<char *> args = {prog};

std::string output_dir;
unsigned verbose = 0;
std::unordered_set<std::string> function_names;
std::unordered_set<std::string> skip_function_names;
std::unordered_set<unsigned> include_edges;
unsigned spec_depth = 0;
unsigned max_parallel = 1;
unsigned rob_size = 10;
unsigned max_traceback = 1;
std::ofstream log_;
LeakageClass leakage_class = LeakageClass::INVALID;
AliasMode alias_mode;
SpectreV1Mode spectre_v1_mode;
SpectreV4Mode spectre_v4_mode;
bool witness_executions = true;
bool partial_executions = false;
bool fast_mode = false;
bool batch_mode = false;
std::optional<unsigned> stb_size;
SyntacticDependencies respect_syntactic_dependencies;
bool use_lookahead = false;
unsigned window_size = std::numeric_limits<unsigned>::max();
bool profile = false;
bool fence_insertion = false;
int semid = -1;
int shmid = -1;
std::vector<std::pair<aeg::Edge::Kind, aeg::ExecMode>> custom_deps;
std::string file_regex;

namespace {
std::optional<std::string> logdir;
int saved_fd = -1;
}

OutputCFGs output_cfgs;

mon::Client client;

SharedDatabase analyzed_functions;


namespace {

void usage(FILE *f = stderr) {
    const char *s = R"=(usage: [option...]
Required options:
--depth, -d <n>      speculation depth
--output, -o <path>  output directory

Options:
--help, -h           show help
--func, -f <name>[,<name>]...   only examine given functions
--skip-func, -F <name>  skip given function
--file <regex>       only analyze files matching given regex
--verbose, -v        verbosity++
--edges, -E          include edges in execution graph output
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
--lookahead=[<bool>] use lookahead during leakage detection
--window <uint>      sliding window size
--log <dir>          redirect stderr to log directory
--profile            enable profiler
--fence=[<bool>]     perform automatic fence insertion
--deps=[<vec>]       set custom dependencies (empty means use default)
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
    /* initialize output directory */
    const auto make_dir = [] (const std::string& path) {
        if (::mkdir(path.c_str(), 0777) < 0 && errno != EEXIST) {
            throw std::system_error(errno, std::generic_category(), util::to_string("mkdir(", path, ")"));
        }
    };
    
    make_dir(output_dir);
    make_dir(output_dir + "/logs");
    make_dir(output_dir + "/tmp");
    make_dir(output_dir + "/lkg");
    make_dir(output_dir + "/logs");
    
    {
        std::stringstream ss;
        ss << output_dir << "/functions.db";
        analyzed_functions.open(ss.str());
    }
    
    std::signal(SIGPIPE, SIG_IGN);
    
    if (logdir) {
        std::stringstream ss;
        ss << *logdir << "/" << ::getpid() << ".log";
        if ((saved_fd = ::open(ss.str().c_str(), O_WRONLY | O_TRUNC | O_CREAT, 0664)) < 0) {
            std::cerr << prog << ": open: " << ss.str() << ": " << std::strerror(errno) << "\n";
        } else if (::dup2(saved_fd, STDERR_FILENO) < 0) {
            std::perror("dup");
        }
    }
}

int parse_args() {
    const char *envvars[] = {"LCM_ARGS", "LCM_EXTRA_ARGS"};
    for (const char *envvar : envvars) {
        if (char *line = std::getenv(envvar)) {
            while (char *s = ::strsep(&line, " ")) {
                if (*s) {
                    args.push_back(s);
                }
            }
        }
    }
    
    char **argv = args.data();
    int argc = args.size();
    int optc;
    
    initialize();
    
    enum Option {
        BEGIN = 256,
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
        LOOKAHEAD,
        WINDOW,
        LOG,
        PROFILE,
        FENCE,
        DEPS,
        SKIP_FUNC,
        FILE_REGEX,
    };
    
    struct option opts[] = {
        {"help", no_argument, nullptr, 'h'},
        {"verbose", no_argument, nullptr, 'v'},
        {"output", required_argument, nullptr, 'o'},
        {"function", required_argument, nullptr, 'f'},
        {"edges", required_argument, nullptr, 'E'},
        {"depth", required_argument, nullptr, 'd'},
        {"jobs", required_argument, nullptr, 'j'},
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
        {"lookahead", optional_argument, nullptr, LOOKAHEAD},
        {"window", required_argument, nullptr, WINDOW},
        {"log", required_argument, nullptr, LOG},
        {"profile", optional_argument, nullptr, PROFILE},
        {"parallel", required_argument, nullptr, 'j'},
        {"fence", optional_argument, nullptr, FENCE},
        {"deps", optional_argument, nullptr, DEPS},
        {"skip-func", required_argument, nullptr, 'F'},
	{"file", required_argument, nullptr, FILE_REGEX},
        {nullptr, 0, nullptr, 0}
    };
    
    while ((optc = getopt_long(argc, argv, "hvo:cef:F:E:d:j:", opts, nullptr)) >= 0) {
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
                
            case 'f':
                function_names.insert(optarg);
                break;
                
            case 'F':
                skip_function_names.insert(optarg);
                break;
                
            case 'E': {
                const char *token;
                while ((token = strsep(&optarg, ",")) != nullptr) {
                    include_edges.insert(aeg::Edge::kind_fromstr(token));
                }
                break;
            }
                
            case 'd':
            case MAX_TRANSIENT:
                spec_depth = std::stoul(optarg);
                break;
                
            case 'j': {
                if (optarg) {
                    max_parallel = std::stoul(optarg);
                }
                {
                    key_t key;
                    if ((key = ::ftok("/lcm", 0)) < 0) {
                        std::cerr << "ftok: invalid path '/lcm'\n";
                    } else if ((semid = ::semget(key, 0,
#ifndef __linux__
                                               SEM_R | SEM_A |
#endif
                                               IPC_CREAT)) < 0) {
                        std::perror("semget");
                    }
                }
                break;
            }
                
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
                    CONCRETE_SRC,
                    COUNT
                };

                const char *keylist[COUNT + 1] = {
                    [PSF] = "psf",
                    [CONCRETE_SRC] = "concrete-src",
                    [COUNT] = nullptr
                };
                
                bool args[COUNT] = {
                    [PSF] = false,
                    [CONCRETE_SRC] = true,
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
                            
                        case CONCRETE_SRC:
                            spectre_v4_mode.concrete_sourced_stores = parse_bool(value);
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
                    use_lookahead = true;
                    spectre_v4_mode.concrete_sourced_stores = false;
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
                        if (*tok == '\0') { continue; }
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
                
            case LOOKAHEAD: {
                use_lookahead = parse_bool_opt(optarg);
                break;
            }
                
            case WINDOW: {
                window_size = std::stoul(optarg);
                break;
            }
                
            case LOG: {
                logdir = optarg;
                break;
            }
                
            case PROFILE: {
                profile = parse_bool_opt(optarg);
                break;
            }
                
            case FENCE: {
                fence_insertion = parse_bool_opt(optarg);
                break;
            }
                
            case DEPS: {
                custom_deps.clear();
                char *tok;
                while ((tok = ::strsep(&optarg, ",")) != nullptr) {
                    // check if has :
                    const char *edge = ::strsep(&tok, ":");
                    const char *mode = tok;
                    assert(edge != nullptr);
                    custom_deps.emplace_back(aeg::Edge::kind_fromstr(edge),
                                          mode == nullptr ? aeg::ExecMode::EXEC : aeg::from_string<aeg::ExecMode>(mode));
                }
                break;
            }
                
	case FILE_REGEX:
	  file_regex = optarg;
	  break;
                
            default:
                usage();
                exit(1);
        }
    }
    
    
    // check
    // check_config();
    initialize_post();
    
    if (use_lookahead) {
        logv(1, "config: using lookahead\n");
    }
    
    return 0;
}

const int parse_args_force = parse_args();

}

void check_config() {
    if (leakage_class == LeakageClass::INVALID) {
        throw util::resume("warning: missing leakage class option (--spectre-v1, --spectre-v4, ...)");
    }
    
    if (spec_depth == 0) {
        throw util::resume("error: no speculation depth specified (-d)");
    }
}

void open_log(const std::string& name) {
    if (!logdir) { return; }
    std::stringstream path;
    path << *logdir << "/" << name << "." << ::getpid() << ".log";
    assert(saved_fd >= 0);
    int log_fd;
    if ((log_fd = ::open(path.str().c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0664)) < 0) {
        std::cerr << prog << ": open: " << path.str() << ": " << std::strerror(errno) << "\n";
        std::exit(1);
    }
    if (::dup2(log_fd, STDERR_FILENO) < 0) {
        std::perror("dup2");
        std::abort();
    }
    ::close(log_fd);
}

void close_log() {
    if (!logdir) { return; }
    assert(saved_fd >= 0);
    ::dup2(saved_fd, STDERR_FILENO);
}
