#pragma once

#include <unordered_map>
#include <string>

#include <z3++.h>

namespace aeg {


class Context {
public:
    Context();
    
    z3::context context;
    const z3::expr TRUE;
    const z3::expr FALSE;
    
    z3::expr make_bool(const std::string& s = "") {
        return context.bool_const(get_name(s).c_str());
    }
    
    z3::expr make_int(const std::string& s = "") {
        return context.int_const(get_name(s).c_str());
    }
    
    z3::expr make_const(const std::string& s, const z3::sort& sort) {
        return context.constant(get_name(s).c_str(), sort);
    }
        
    z3::expr bool_val(bool b) const {
        return b ? TRUE : FALSE;
    }
    
    operator z3::context& () {
        return context;
    }
    
private:
    std::unordered_map<std::string, unsigned> next;
    
    std::string get_name(const std::string& s) {
        unsigned& idx = next[s];
        return s + std::to_string(idx++);
    }
};


}
