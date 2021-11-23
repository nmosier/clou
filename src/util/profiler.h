#pragma once

#include <signal.h>

#include <gperftools/profiler.h>

class Profiler {
public:
    Profiler(const std::string& path) {
        assert(!lock);
        lock = true;
        
        if (oact == nullptr) {
            oact = signal(SIGINT, [] (int sig) {
                if (lock) {
                    stop();
                }
            });
        }
        
        ProfilerStart(path.c_str());
    }
    
    static void stop() {
        assert(lock);
        ProfilerStop();
        lock = false;
    }
    
    ~Profiler() {
        if (lock) {
            stop();
        }
    }
private:
    static inline bool lock = false;
    static inline sig_t oact = nullptr;
};
