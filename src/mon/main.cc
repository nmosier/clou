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

#include <curses.h>

#include "mon/proto.h"

const char *prog;



namespace {
int argc;
char **argv;

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
    
    int fd;
    if ((fd = ::open(path, O_RDONLY)) < 0) {
        perror_exit("open");
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

struct Job: Component {
    std::string name;
    
    virtual void display() override {
        ::addstr(name.c_str());
    }
    
    Job(const std::string& name): name(name) {}
};

struct RunningJob: Job {
    RunningDuration duration;
    
    virtual void display() override {
        Job::display();
        ::addstr(" ");
        duration.display();
    }
    
    RunningJob(const std::string& name): Job(name) {}
};

struct CompletedJob: Job {
    Duration duration;
    
    CompletedJob(const RunningJob& job): Job(job), duration(job.duration.duration()) {}
};

template <typename Subcomponent>
struct ComponentList: Component {
    static_assert(std::is_base_of<Component, Subcomponent>(), "subcomponent must inherit from Component");
    using Vec = std::vector<Subcomponent>;
    
    Vec vec;
    std::string sep = "\n";
    
    virtual void display() override {
        for (auto it = vec.begin(); it != vec.end(); ++it) {
            if (it != vec.begin()) {
                ::addstr(sep.c_str());
            }
            it->display();
        }
    }
};

struct Monitor: Component {
    /* Control stuff */
    int fd;
    std::unordered_set<std::string> analyzed_functions;

    /* Display stuff */
    ComponentList<RunningJob> running_jobs;
    ComponentList<CompletedJob> completed_jobs;
    
    Monitor(int fd): fd(fd) {}
    
    virtual void display() override {
        ::addstr("RUNNING:\n");
        running_jobs.display();
        ::addstr("\n\nCOMPLETED:\n");
        completed_jobs.display();
        ::addstr("\n");
    }
    
    void run();
    
private:
    void run_body();
    
    void handle_func_started(const mon::FunctionStarted& msg);
    void handle_func_completed(const mon::FunctionCompleted& msg);
};


void Monitor::run() {
    struct pollfd pfd = {
        .fd = fd,
        .events = POLLIN,
    };
    while (true) {
        const int poll_res = ::poll(&pfd, 1, 0);
        if (poll_res < 0) {
            perror_exit("poll");
        }
        if (poll_res > 0) {
            if (pfd.revents == POLLIN) {
                run_body();
            } else {
                std::cerr << "error: unexpected poll event on fd\n";
                std::abort();
            }
        }
        
        // update screen
        static unsigned i = 0;
        ::clear();
        ::printw("%u\n", i++);
        display();
        ::refresh();
    }
}

void Monitor::run_body() {
    mon::Message msg;
    if (!msg.ParseFromFileDescriptor(fd)) {
        std::cerr << "warning: bad message\n";
        return;
    }
    
    switch (msg.message_case()) {
        case mon::Message::kFuncStarted:
            handle_func_started(msg.func_started());
            break;
            
        case mon::Message::kFuncCompleted:
            handle_func_completed(msg.func_completed());
            break;
            
        default: std::abort();
    }
}

void Monitor::handle_func_started(const mon::FunctionStarted& msg) {
    running_jobs.vec.push_back(RunningJob(msg.func().name()));
}

void Monitor::handle_func_completed(const mon::FunctionCompleted& msg) {
    const auto end = running_jobs.vec.end();
    const auto it = std::find_if(running_jobs.vec.begin(), end, [&] (const RunningJob& job) -> bool {
        return job.name == msg.func().name();
    });
    if (it == end) {
        std::cerr << "warning: received 'completed job' for job that wasn't running\n";
        return;
    }
    
    RunningJob job = *it;
    running_jobs.vec.erase(it);
    completed_jobs.vec.emplace_back(job);
}


void server(int fd) {
    ::initscr();
    Monitor monitor {fd};
    monitor.run();
    
}
