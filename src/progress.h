#pragma once

#include <cassert>
#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>

class Progress {
public:
    Progress(): os(nullptr) {}
    
    Progress(std::size_t total, const std::string& s = "", std::ostream& os = std::cerr, unsigned precision = 2): s(s), os(&os), cur(0), total(std::max(total, 1UL)), precision(precision), scale(std::pow(10, precision)), start(now()) {
        update();
    }
    
    Progress& operator++() {
        if (cur < total) {
            const auto old = raw();
            ++cur;
            const auto new_ = raw();
            if (old != new_) {
                update();
            }
        }
        return *this;
    }
    
    void update() const {
        char *buf = new char [precision + 8];
        sprintf(buf, "%*.*f%%", precision + 1, std::min<int>(precision - 2, 0), 100.0 * raw() / scale);
        *os << "\r" << title() << buf;
        delete[] buf;
    }
    
    void done() const {
        TimePoint stop = now();
        std::chrono::duration<float> elapsed_sec = stop - start;
        float secs = elapsed_sec.count();
        const char *unit = "s";
        if (secs < 1) {
            secs *= 1000;
            unit = "ms";
        }
        char buf[128];
        sprintf(buf, "%.1f", secs);
        *os << "\r" << title() << buf << unit << "\n";
    }
    
private:
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

    std::string s;
    std::ostream *os;
    std::size_t cur;
    std::size_t total;
    unsigned precision;
    unsigned scale;
    TimePoint start;
    
    unsigned raw() const {
        return scale * cur / total;
    }
    
    static TimePoint now() {
        return std::chrono::steady_clock::now();
    }
    
    std::string title() const {
        if (!s.empty()) {
            return s + ": ";
        } else {
            return "";
        }
    }
};

class Count {
public:
    Count(std::ostream& os = std::cerr, std::size_t count = 0): os(&os), count(count) {}
    
    void update() const {
        *os << "\r" << count;
    }
    
    void done() const {
        *os << "\n";
    }
    
    Count& operator++() {
        ++count;
        update();
        return *this;
    }
    
private:
    std::ostream *os;
    std::size_t count;
};
