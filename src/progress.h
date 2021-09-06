#pragma once

#include <cassert>
#include <iostream>
#include <iomanip>

class Progress {
public:
    Progress(): os(nullptr) {}
    
    Progress(std::ostream& os, std::size_t total, unsigned precision = 2):
    os(&os), cur(0), total(total), precision(precision), scale(std::pow(10, precision)) {
        update();
    }
    
    Progress(std::size_t total, unsigned precision = 2): os(&std::cerr), cur(0), total(total), precision(precision), scale(std::pow(10, precision)) {}
    
    Progress& operator++() {
        // assert(cur < total);
        const auto old = raw();
        ++cur;
        const auto new_ = raw();
        if (old != new_) {
            update();
        }
        return *this;
    }
    
    void update() const {
        char *buf = new char [precision + 8];
        sprintf(buf, "%*.*f%%", precision + 1, std::min<int>(precision - 2, 0), 100.0 * raw() / scale);
        *os << "\r" << buf;
        if (cur == total) {
            *os << "\n";
        }
        delete[] buf;
    }
    
private:
    std::ostream *os;
    std::size_t cur;
    std::size_t total;
    unsigned precision;
    unsigned scale;
    
    unsigned raw() const {
        return scale * cur / total;
    }
};
