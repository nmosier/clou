#pragma once

#include <functional>
#include <deque>
#include <mutex>
#include <thread>
#include <unistd.h>

class thread_work_queue {
public:
    using Function = std::function<void (void)>;
    
    void push(const Function& job) { queue.push_front(job); }
    
    void run(unsigned num_threads) {
        for (unsigned i = 0; i < num_threads; ++i) {
            launch();
        }
        for (unsigned i = 0; i < num_threads; ++i) {
            thds.back().join();
            thds.pop_back();
        }
    }
    
private:
    std::mutex mut;
    std::deque<Function> queue;
    std::vector<std::thread> thds;
    
    void launch() {
        thds.emplace_back([&] {
            while (true) {
                Function job;
                {
                    std::lock_guard<std::mutex> lock {mut};
                    if (queue.empty()) {
                        return;
                    }
                    
                    job = queue.back();
                    queue.pop_back();
                }
                job();
            }
        });
    }
};
