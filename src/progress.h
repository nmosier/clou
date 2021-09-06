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
    
    Progress& operator++() {
        assert(cur < total);
        const auto old = raw();
        ++cur;
        const auto new_ = raw();
        if (old != new_) {
            update();
        }
        return *this;
    }
    
    void update() const {
        *os << "\r" << std::setprecision(precision) << 100.0 * raw() / scale;
        if (cur == total) {
            *os << "\n";
        }
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
