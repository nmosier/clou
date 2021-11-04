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
#include <mutex>
#include <thread>

#include <curses.h>

#include "mon/proto.h"

const char *prog;



namespace {
int argc;
char **argv;

const char *path;

void perror_exit(const char *s) {
    fprintf(stderr, "%s: %s: %s\n", prog, s, std::strerror(errno));
    std::exit(EXIT_FAILURE);
}

template <typename... Args>
void error(const char *fmt, Args&&... args) {
    fprintf(stderr, "%s: ", prog);
    fprintf(stderr, fmt, std::forward<Args>(args)...);
    fprintf(stderr, "\n");
    std::exit(EXIT_FAILURE);
}

}

void usage(FILE *f) {
    const char *s = R"=(usage: %s [-h] <fifo_path>
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
    
    const char *optstr = "h";
    int optchar;
    while ((optchar = getopt(argc, argv, optstr)) >= 0) {
        switch (optchar) {
            case 'h':
                usage(stdout);
                return EXIT_SUCCESS;
                
            default:
                usage(stderr);
                return EXIT_FAILURE;
        }
    }
    
    const char *path = nextarg();
    ::path = path;
    
    int fd;
    if ((fd = ::socket(PF_LOCAL, SOCK_STREAM, 0)) < 0) {
        perror_exit("socket");
    }
    
    // cleanup
    std::atexit([] () {
        ::unlink(::path);
    });
    
    struct sockaddr_un addr;
    addr.sun_family = AF_LOCAL;
    ::strlcpy(addr.sun_path, path, sizeof(addr.sun_path));
    if (::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0) {
        perror_exit("bind");
    }
    if (::listen(fd, 16) < 0) {
        perror_exit("listen");
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
    
    std::string desc;
    
    virtual void display() override {
        ::addstr(desc.c_str());
    }
    
    Duration(float s) {
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
    
    virtual void display() override {
        ::addstr(name.c_str());
    }
    
    Job(const std::string& name): name(name) {}
};

struct RunningJob: Job {
    RunningDuration duration;
    Progress progress;
    
    virtual void display() override {
        Job::display();
        ::addstr(" ");
        duration.display();
        ::addstr(" ");
        progress.display();
    }
    
    RunningJob(const std::string& name): Job(name) {}
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
    
    iterator find_if(std::function<bool (const Subcomponent&)> pred) {
        return std::find_if(begin(), end(), pred);
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

    /* Display stuff */
    unsigned msgs = 0;
    using RunningJobList = ComponentList<RunningJob>;
    RunningJobList running_jobs {16};
    using CompletedJobList = ComponentList<CompletedJob>;
    CompletedJobList completed_jobs {16};
    
    /** NOTE: \p server_sock must already be set to listening. */
    Monitor(int server_sock): server_sock(server_sock) {}
    
    virtual void display() override {
        static unsigned i = 0;
        ::printw("%u\n", i++);
        ::printw("MESSAGES: %u\n", msgs);
        ::printw("CLIENTS: %zu\n", client_thds.size());
        ::addstr("RUNNING:\n");
        running_jobs.display();
        ::addstr("\n\nCOMPLETED:\n");
        completed_jobs.display();
        ::addstr("\n");
    }
    
    void run();
    
private:
    void run_body(int client_sock);
    
    void handle_func_started(const mon::FunctionStarted& msg);
    void handle_func_completed(const mon::FunctionCompleted& msg);
    void handle_func_progress(const mon::FunctionProgress& msg);
};


void Monitor::run() {
    // listen for clients
    listen_thd = std::thread {
        [&] () {
            while (true) {
                struct sockaddr_un addr;
                socklen_t addrlen = sizeof(addr);
                int client_sock;
                if ((client_sock = ::accept(server_sock, reinterpret_cast<struct sockaddr *>(&addr), &addrlen)) < 0) {
                    std::perror("accept");
                } else {
                    std::unique_lock<std::mutex> lock {mutex};
                    client_thds.emplace_back([this] (int client_sock) {
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
                                this->run_body(pfd.fd);
                            }
                            if ((pfd.revents & POLLHUP)) {
                                ::close(pfd.fd);
                                return;
                            }
                            if ((pfd.revents & POLLNVAL)) {
                                error("poll: invalid socket");
                            }
                            if ((pfd.revents & POLLERR)) {
                                std::cerr << prog << ": error on socket, closing\n";
                                ::close(pfd.fd);
                                return;
                            }
                        }
                    }, client_sock);
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
                ::napms(100);
            }
        }
    };
    
    listen_thd.join();
    for (std::thread& thd : client_thds) {
        thd.join();
    }
    display_thd.join();
}

void Monitor::run_body(int client_sock) {
    ++msgs;
    mon::Message msg;
    
    if (!msg.ParseFromFileDescriptor(client_sock)) {
        std::cerr << "warning: bad message\n";
        return;
    }
    
    ++msgs;
    
    std::unique_lock<std::mutex> lock {mutex};
    
    switch (msg.message_case()) {
        case mon::Message::kFuncStarted:
            handle_func_started(msg.func_started());
            break;
            
        case mon::Message::kFuncCompleted:
            handle_func_completed(msg.func_completed());
            break;
            
        case mon::Message::kFuncProgress:
            handle_func_progress(msg.func_progress());
            break;
            
        case mon::Message::MESSAGE_NOT_SET:
            break;
            
        default: std::abort();
    }
}

void Monitor::handle_func_started(const mon::FunctionStarted& msg) {
    running_jobs.vec.push_back(RunningJob(msg.func().name()));
}

void Monitor::handle_func_completed(const mon::FunctionCompleted& msg) {
    const auto it = running_jobs.find_if([&] (const RunningJob& job) -> bool {
        return job.name == msg.func().name();
    });
    if (it == running_jobs.end()) {
        std::cerr << "warning: received 'completed job' for job that wasn't running\n";
        return;
    }
    
    RunningJob job = *it;
    running_jobs.vec.erase(it);
    completed_jobs.vec.emplace_back(job);
}

void Monitor::handle_func_progress(const mon::FunctionProgress& msg) {
    const auto it = running_jobs.find_if([&] (const RunningJob& job) -> bool {
        return job.name == msg.func().name();
    });
    if (it == running_jobs.end()) {
        std::cerr << "warning: received 'job progress' for job that wasn't running\n";
    }
    it->progress.frac = msg.frac();
}

void server(int server_sock) {
    ::initscr();
    
    // listen for incoming connections
    
    Monitor monitor {server_sock};
    
    monitor.run();
    
}
