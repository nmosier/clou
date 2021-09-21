#pragma once

#include <deque>
#include <vector>
#include <system_error>
#include <unistd.h>
#include <sys/wait.h>
#include <unordered_set>
#include <functional>

#include "progress.h"

class fork_work_queue {
public:
    using Function = std::function<int (void)>;
    
    fork_work_queue(unsigned num_processes, bool report_progress = false): num_processes(num_processes), report_progress(report_progress) {}

    template <typename OutputIt>
    OutputIt run(OutputIt out) {
        Progress progress;
        if (report_progress) {
            progress = Progress(queue.size());
        }
        while (busy() || !empty()) {
            if (!saturated() && !empty()) {
                launch();
            } else {
                *out++ = join();
                if (report_progress) {
                    ++progress;
                }
            }
        }
        if (report_progress) {
            progress.done();
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
    const bool report_progress;
    std::deque<Function> queue;
    std::unordered_map<pid_t, std::size_t> running;
    
    std::pair<Function, std::size_t> pop() {
        const auto item = queue.back();
        queue.pop_back();
        return std::make_pair(item, queue.size());
    }
    
    void launch() {
        const auto job = pop();
        const auto pid = ::fork();
        if (pid == -1) {
            throw std::system_error(errno, std::system_category());
        } else if (pid == 0) {
            const auto res = job.first();
            std::exit(res);
        }
        running.emplace(pid, job.second);
    }
    
    std::pair<int, std::size_t> join() {
        while (true) {
            int status;
            pid_t pid;
            do {
                pid = ::wait(&status);
            } while (pid < 0 && errno == EINTR);
            if (pid < 0) {
                throw std::system_error(errno, std::system_category());
            } else {
                const auto res = std::make_pair(status, running.at(pid));
                running.erase(pid);
                return res;
            }
        }
    }
};

