#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <utility>
#include <sstream>
#include <chrono>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <mutex>
#include <thread>
#include <algorithm>
#include <variant>
#include <fstream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>

#include <curses.h>

#include "mon/proto.h"
#include "util/timer.h"
#include "util/algorithm.h"

#define CLOU_ROOT "/clou"

const char *prog;

namespace {
int argc;
char **argv;

const char *fifo_path = nullptr;
const char *table_path = nullptr;
const char *sem_path = nullptr;
int semid = -1;
std::optional<unsigned> num_threads;


void cleanup() {
    if (fifo_path != nullptr) {
        ::unlink(fifo_path);
        fifo_path = nullptr;
    }
    if (semid >= 0) {
        semid = -1;
    }
}

std::ofstream table_os;

Timer timer;
std::ostream& table_log() {
    return table_os << "[" << timer.get_str() << "] ";
}

void perror_exit(const char *s) {
    fprintf(stderr, "%s: %s: %s\n", prog, s, std::strerror(errno));
    std::exit(EXIT_FAILURE);
}

#define error(fmt, ...) do { \
std::fprintf(stderr, "%s: ", prog); \
std::fprintf(stderr, fmt __VA_OPT__(,) __VA_ARGS__); \
std::fprintf(stderr, "\n"); \
std::exit(EXIT_FAILURE); \
} while (false)

}

void usage(FILE *f) {
    const char *s = R"=(usage: %s [-h] [-f <fifo>=$FIFO] [-t <path>]
)=";
    ;
    fprintf(f, s, prog);
}

char *nextarg() {
    if (optind == argc) {
        usage(stderr);
        std::exit(EXIT_FAILURE);
    }
    return argv[optind++];
}

void server(int fd);

int main(int argc, char *argv[]) {
    ::argc = argc;
    ::argv = argv;
    ::prog = argv[0];
    
    ::signal(SIGINT, [] (int) {
        std::exit(EXIT_SUCCESS);
    });
        
    const char *optstr = "hf:t:j:";
    int optchar;
    while ((optchar = getopt(argc, argv, optstr)) >= 0) {
        switch (optchar) {
            case 'h':
                usage(stdout);
                return EXIT_SUCCESS;
                
            case 'f':
                ::fifo_path = fifo_path = optarg;
                break;
                
            case 't':
                table_path = optarg;
                break;
                
            case 'j':
                num_threads = std::stoul(optarg);
                break;
                
            default:
                usage(stderr);
                return EXIT_FAILURE;
        }
    }
    
    int fd;
    if ((fd = ::socket(PF_LOCAL, SOCK_STREAM, 0)) < 0) {
        perror_exit("socket");
    }
    
    // cleanup
    std::atexit([] () { cleanup(); });
        
    struct sockaddr_un addr;
    addr.sun_family = AF_LOCAL;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", fifo_path);
    if (::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0) {
        perror_exit("bind");
    }
    if (::listen(fd, 16) < 0) {
        perror_exit("listen");
    }
    
    // open semaphore
    if (num_threads) {
      sem_path = CLOU_ROOT;
        key_t key;
        union semun {
                int     val;            /* value for SETVAL */
                struct  semid_ds *buf;  /* buffer for IPC_STAT & IPC_SET */
                u_short *array;         /* array for GETALL & SETALL */
        } semun = {.val = static_cast<int>(*num_threads)};
        if ((key = ::ftok(sem_path, 0)) < 0) {
            std::perror("ftok");
        } else if ((semid = ::semget(key, 1,
#ifndef __linux__
                                     SEM_R | SEM_A |
#endif
                                     IPC_CREAT)) < 0) {
            std::perror("semget");
        } else if (::semctl(semid, 0, SETVAL, semun) < 0) {
            std::perror("semctl");
        } else {
            std::cerr << "successfully opened semaphore for " << *num_threads << " threads\n";
        }
    }

    server(fd);
    
    ::close(fd);
}


struct Component {
    virtual void display() = 0;
    
    virtual ~Component() {}
};

struct Duration: Component {
    using Clock = std::chrono::system_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    
    float secs;
    std::string desc;
    
    virtual void display() override {
        ::addstr(desc.c_str());
    }
    
    Duration(float s): secs(s) {
        float t = s;
        const char *unit = "s";
        if (t < 1) {
            t *= 1e3;
            unit = "ms";
        }
        char buf[128];
        sprintf(buf, "%.1f", t);
        std::stringstream ss;
        ss << buf << unit;
        desc = ss.str();
    }
};

struct RunningDuration: Component {
    Duration::TimePoint start;
    
    float elapsed() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Duration::Clock::now() - start).count() / 1e3;
    }
    
    Duration duration() const {
        return Duration(elapsed());
    }
    
    virtual void display() override {
        duration().display();
    }
    
    RunningDuration(): start(Duration::Clock::now()) {}
};

struct Progress: Component {
    float frac = 0.;
    
    virtual void display() override {
        ::printw("%.1f%%", frac * 100.);
    }
};

struct Job: Component {
    std::string name;
    pid_t pid;
    std::unordered_map<std::string, std::string> properties;
    
    virtual void display() override {
        ::printw("%8d ", pid);
        ::addstr(name.c_str());
        std::stringstream ss;
        ss << "{";
        for (auto it = properties.begin(); it != properties.end(); ++it) {
            if (it != properties.begin()) {
                ss << ", ";
            }
            ss << it->first << ": " << it->second;
        }
        ss << "}";
        ::printw(" %s", ss.str().c_str());
    }
    
    Job(const std::string& name, pid_t pid): name(name), pid(pid) {}
};

using owner_t = int;

struct RunningJob: Job {
    owner_t owner;
    RunningDuration duration;
    Progress progress;
    std::string step;
    
    virtual void display() override {
        ::printw("%8d ", pid);
        Job::display();
        ::addstr(" ");
        duration.display();
        ::addstr(" ");
        progress.display();
        ::addstr(" ");
        Duration(duration.elapsed() / progress.frac - duration.elapsed()).display();
        if (!step.empty()) {
            ::printw(" %s", step.c_str());
        }
    }
    
    RunningJob(const std::string& name, owner_t owner, pid_t pid): Job(name, pid), owner(owner) {}
};

struct CompletedJob: Job {
    Duration duration;
    
    CompletedJob(const RunningJob& job): Job(job), duration(job.duration.duration()) {}
    
    virtual void display() override {
        Job::display();
        ::addstr(" ");
        duration.display();
    }
};

template <typename Subcomponent>
struct ComponentList: Component {
    static_assert(std::is_base_of<Component, Subcomponent>(), "subcomponent must inherit from Component");
    using Vec = std::vector<Subcomponent>;
    
    Vec vec;
    std::string sep = "\n";
    unsigned limit;
    
    ComponentList(unsigned limit): limit(limit) {}
    
    virtual void display() override {
        const unsigned cap = std::min<unsigned>(limit, vec.size());
        for (unsigned i = 0; i < cap; ++i) {
            if (i != 0) {
                ::addstr(sep.c_str());
            }
            vec[i].display();
        }
        if (cap < vec.size()) {
            if (cap > 0) {
                ::addstr(sep.c_str());
            }
            ::printw("(+%u more)", vec.size() - cap);
        }
    }
    
    using iterator = typename Vec::iterator;
    iterator begin() { return vec.begin(); }
    iterator end() { return vec.end(); }
    
    using Pred = std::function<bool (const Subcomponent&)>;
    iterator find_if(Pred pred) {
        return std::find_if(begin(), end(), pred);
    }
    
    void remove_if(Pred pred) {
        std::remove_if(begin(), end(), pred);
    }
};

struct Monitor: Component {
    /* Control stuff */
    std::mutex mutex;
    int server_sock;
    std::thread listen_thd;
    std::vector<std::thread> client_thds;
    std::thread display_thd;
    std::unordered_set<std::string> analyzed_functions;
    RunningDuration clock;
    
    struct ThreadStats {
        unsigned long num_samples = 0;
        unsigned long total_threads = 0;
        
        void update(unsigned num_threads) {
            ++num_samples;
            total_threads += num_threads;
        }
        
        float average_threads() const {
            return static_cast<float>(total_threads) / static_cast<float>(num_samples);
        }
        
        std::string str() const {
            std::stringstream ss;
            ss << "average threads: " << average_threads();
            return ss.str();
        }
    };
    
    ThreadStats thread_stats;

    /* Display stuff */
    unsigned msgs = 0;
    using RunningJobList = ComponentList<RunningJob>;
    RunningJobList running_jobs {48};
    using CompletedJobList = ComponentList<CompletedJob>;
    CompletedJobList completed_jobs {16};
    using AbortedJobList = ComponentList<CompletedJob>;
    AbortedJobList aborted_jobs {16};
    
    /** NOTE: \p server_sock must already be set to listening. */
    Monitor(int server_sock, const char *table_path): server_sock(server_sock) {
        if (table_path != nullptr) {
            table_os.open(table_path);
        }
    }
    
    virtual void display() override {
        static unsigned i = 0;
        ::printw("%u\n", i++);
        ::printw("MESSAGES: %u\n", msgs);
        ::printw("CLIENTS: %zu\n", client_thds.size());
        ::addstr("RUNNING:\n");
        std::sort(running_jobs.vec.begin(), running_jobs.vec.end(), [] (const RunningJob& a, const RunningJob& b) -> bool {
            return a.duration.elapsed() > b.duration.elapsed();
        });
        running_jobs.display();
        ::addstr("\n\nCOMPLETED:\n");
        std::sort(completed_jobs.vec.begin(), completed_jobs.vec.end(), [] (const CompletedJob& a, const CompletedJob& b) -> bool {
            return a.duration.secs > b.duration.secs;
        });
        completed_jobs.display();
        ::addstr("\nABORTED:\n");
        aborted_jobs.display();
        ::printw("\nANALYZED: %zu\n", analyzed_functions.size());
        ::printw("THREADS: %d\n",  *num_threads - ::semctl(semid, 0, GETVAL));
        thread_stats.update(*num_threads - ::semctl(semid, 0, GETVAL));
        ::printw("%s\n", thread_stats.str().c_str());
    }
    
    void run();
    
private:
    bool run_body(FILE *client_f, int owner, ::pid_t pid);
    
    void handle_func_started(const mon::FunctionStarted& msg, int owner, ::pid_t pid);
    void handle_func_completed(const mon::FunctionCompleted& msg, pid_t pid);
    void handle_func_progress(const mon::FunctionProgress& msg, pid_t pid);
    void handle_funcs_analyzed(const mon::FunctionsAnalyzed& msg);
    void handle_func_step(const mon::FunctionStep& msg, pid_t pid);
    void handle_func_properties(const mon::FunctionProperties& msg, pid_t pid);
    
    template <typename T>
    bool client_read(FILE *f, T *buf, std::size_t count, bool report_eof) const {
        if (std::fread(buf, sizeof(T), count, f) != count) {
            if (std::feof(f)) {
                if (report_eof) {
                    std::cerr << "warning: unexpected client EOF\n";
                }
            } else {
                std::perror("fread");
            }
            return false;
        }
        return true;
    }
    
    template <typename Msg>
    bool parse(FILE *client_f, Msg& msg) {
        uint32_t buflen;
        if (!client_read(client_f, &buflen, 1, false)) { return false; }
        buflen = ntohl(buflen);
        std::vector<char> buf;
        buf.resize(buflen);
        if (!client_read(client_f, buf.data(), buflen, true)) { return false; }
        if (!msg.ParseFromArray(buf.data(), buf.size())) {
            std::cerr << "warning: bad message\n";
            return false;
        }
        ++msgs;
        return true;
    }
};


void Monitor::run() {
    // listen for clients
    listen_thd = std::thread {
        [&] () {
            int id = 0;
            while (true) {
                struct sockaddr_un addr;
                socklen_t addrlen = sizeof(addr);
                int client_sock;
                if ((client_sock = ::accept(server_sock, reinterpret_cast<struct sockaddr *>(&addr), &addrlen)) < 0) {
                    std::perror("accept");
                } else {
                    std::unique_lock<std::mutex> lock {mutex};
                    client_thds.emplace_back([this] (int client_sock, int id) {
                        FILE *client_f;
                        if ((client_f = ::fdopen(client_sock, "r+")) == nullptr) {
                            perror_exit("fdopen");
                        }
                        
                        // read client connect packet
                        pid_t pid;
                        {
                            mon::ClientConnect msg;
                            if (!parse(client_f, msg)) { goto cleanup; }
                            pid = msg.pid();
                        }
                        
                        while (true) {
                            struct pollfd pfd = {
                                .fd = client_sock,
                                .events = POLLIN,
                            };
                            if (::poll(&pfd, 1, -1) < 0) {
                                perror_exit("poll");
                            }
                            
                            /* check flags */
                            if ((pfd.revents & POLLIN)) {
                                if (!this->run_body(client_f, id, pid)) {
                                    break;
                                }
                            } else {
                                if ((pfd.revents & POLLHUP)) {
                                    break;
                                }
                                if ((pfd.revents & POLLNVAL)) {
                                    error("poll: invalid socket");
                                }
                                if ((pfd.revents & POLLERR)) {
                                    std::cerr << prog << ": error on socket, closing\n";
                                    break;
                                }
                            }
                        }
                        
                        cleanup:
                            
                        std::fclose(client_f);
                        
                        /* cleanup monitor state */
                        {
                            std::unique_lock<std::mutex> lock {mutex};
                            for (auto it = running_jobs.begin(); it != running_jobs.end(); ) {
                                if (it->owner == id) {
                                    const auto job = *it;
                                    it = running_jobs.vec.erase(it);
                                    aborted_jobs.vec.push_back(job);
                                } else {
                                    ++it;
                                }
                            }
                        }
                    }, client_sock, id++);
                }
            }
        }
    };
    
    // display thread
    display_thd = std::thread {
        [this] () {
            while (true) {
                ::clear();
                {
                    std::unique_lock<std::mutex> lock {mutex};
                    this->display();
                }
                ::refresh();
                ::napms(1000);
            }
        }
    };
    
    listen_thd.join();
    for (std::thread& thd : client_thds) {
        thd.join();
    }
    display_thd.join();
}

bool Monitor::run_body(FILE *client_f, int owner, pid_t pid) {
    mon::Message msg;
    if (!parse(client_f, msg)) { return false; }
    
    std::unique_lock<std::mutex> lock {mutex};
    
    switch (msg.message_case()) {
        case mon::Message::kFuncStarted:
            handle_func_started(msg.func_started(), owner, pid);
            break;
            
        case mon::Message::kFuncCompleted:
            handle_func_completed(msg.func_completed(), pid);
            break;
            
        case mon::Message::kFuncProgress:
            handle_func_progress(msg.func_progress(), pid);
            break;
            
        case mon::Message::kFuncsAnalyzed:
            handle_funcs_analyzed(msg.funcs_analyzed());
            break;
            
        case mon::Message::kFuncStep:
            handle_func_step(msg.func_step(), pid);
            break;
            
        case mon::Message::kFuncProps:
            handle_func_properties(msg.func_props(), pid);
            break;
            
        case mon::Message::MESSAGE_NOT_SET:
            break;
            
        default: std::abort();
    }
    
    return true;
}

void Monitor::handle_func_started(const mon::FunctionStarted& msg, int owner, ::pid_t pid) {
    running_jobs.vec.emplace_back(msg.func().name(), owner, pid);
    table_log() << "START" << " " << clock.duration().secs << " " << pid << " " << msg.func().name() << "\n";
}

void Monitor::handle_func_completed(const mon::FunctionCompleted& msg, pid_t pid) {
    const auto it = running_jobs.find_if([&] (const RunningJob& job) -> bool {
        return job.pid == pid && job.name == msg.func().name();
    });
    if (it == running_jobs.end()) {
        std::cerr << "warning: received 'completed job' for job that wasn't running\n";
        return;
    }
    
    RunningJob job = *it;
    running_jobs.vec.erase(it);
    completed_jobs.vec.emplace_back(job);
    table_log() << "COMPLETE" << " " << clock.duration().secs << " " << pid << " " << msg.func().name() << "\n";
}

void Monitor::handle_func_progress(const mon::FunctionProgress& msg, pid_t pid) {
    for (RunningJob& job : running_jobs.vec) {
        if (pid == job.pid && job.name == msg.func().name()) {
            job.progress.frac = msg.frac();
        }
    }
}

void Monitor::handle_funcs_analyzed(const mon::FunctionsAnalyzed& msg) {
    util::transform(msg.funcs(), std::inserter(analyzed_functions, analyzed_functions.end()), [] (const mon::Function& func) -> std::string {
        return func.name();
    });
    table_log() << "ANALYZE" << " " << clock.duration().secs << " " << msg.funcs().size() << "\n";
}

void Monitor::handle_func_step(const mon::FunctionStep& msg, pid_t pid) {
    for (RunningJob& job : running_jobs.vec) {
        if (pid == job.pid && job.name == msg.func().name()) {
            job.step = msg.step();
        }
    }
}

void Monitor::handle_func_properties(const mon::FunctionProperties& msg, pid_t pid) {
    for (RunningJob& job : running_jobs.vec) {
        if (pid == job.pid && job.name == msg.func().name()) {
            for (const auto& p : msg.properties()) {
                job.properties[p.first] = p.second;
            }
        }
    }
    table_log() << "PROPERTIES" << " " << pid << " " << msg.func().name() << " ";
    for (auto it = msg.properties().begin(); it != msg.properties().end(); ++it) {
        if (it != msg.properties().begin()) {
            table_os << ",";
        }
        table_os << "{" << it->first << "=" << it->second << "}";
    }
    table_os << "\n";
}


void server(int server_sock) {
    ::initscr();
    
    // listen for incoming connections
    
    Monitor monitor {server_sock, table_path};
    
    monitor.run();
    
}
