#pragma once

#include <deque>
#include <vector>
#include <system_error>
#include <unistd.h>
#include <sys/wait.h>
#include <unordered_set>
#include <functional>

class fork_work_queue {
public:
    using Function = std::function<void (void)>;
    
    fork_work_queue(unsigned num_processes): num_processes(num_processes) {}

    template <typename OutputIt>
    OutputIt run(OutputIt out) {
        while (busy() || !empty()) {
            if (!saturated() && !empty()) {
                launch();
            } else {
                *out++ = join();
            }
        }
        return out;
    }
    
    void push(Function item) {
        queue.push_front(item);
    }
    
    bool empty() const { return queue.empty(); }
    bool busy() const { return !running.empty(); }
    bool saturated() const { return running.size() == num_processes; }
    
private:
    const unsigned num_processes;
    std::deque<Function> queue;
    std::unordered_set<pid_t> running;
    
    Function pop() {
        const auto item = queue.back();
        queue.pop_back();
        return item;
    }
    
    void launch() {
        const auto job = pop();
        const auto pid = ::fork();
        if (pid == -1) {
            throw std::system_error(errno, std::system_category());
        } else if (pid == 0) {
            job();
            std::exit(0);
        }
        running.insert(pid);
    }
    
    int join() {
        while (true) {
            int status;
            pid_t pid;
            do {
                pid = ::wait(&status);
            } while (pid < 0 && errno == EINTR);
            if (pid < 0) {
                throw std::system_error(errno, std::system_category());
            } else if (running.erase(pid) > 0) {
                return status;
            }
        }
    }
};

