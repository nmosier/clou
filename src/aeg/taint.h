#pragma once

#include <z3++.h>

namespace aeg {

class Taint {
public:
    bool get() const { return value; }
    void set(bool value) { this->value = value; }
    operator bool() const { return get(); }
private:
    bool value = true;
};

}
