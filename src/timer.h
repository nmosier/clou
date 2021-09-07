#pragma once

#include <string>
#include <chrono>
#include <iostream>

class Timer {
public:
    Timer(std::ostream& os = std::cerr): os(&os) {
        start_();
    }
    
    ~Timer() {
        stop_();
        show();
    }

private:
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
    std::ostream *os;
    TimePoint start;
    TimePoint stop;

    static TimePoint now() { return std::chrono::steady_clock::now(); }
    
    void start_() {
        start = now();
    }
    
    void stop_() {
        stop = now();
    }
    
    void show() const {
        std::chrono::duration<float> elapsed_sec = stop - start;
        float secs = elapsed_sec.count();
        const char *unit = "s";
        if (secs < 1) {
            secs *= 1000;
            unit = "ms";
        }
        char buf[128];
        sprintf(buf, "%.1f", secs);
        *os << buf << unit << "\n";
    }
};
