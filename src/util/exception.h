#pragma once

#include <string>
#include <exception>
#include <system_error>

namespace util {

struct resume: std::exception {
    virtual const char *what() const noexcept {
        return msg.c_str();
    }
    
    std::string msg;
    
    resume(const std::string& msg): msg(msg) {}
};

inline std::system_error syserr(const std::string& what = "") {
    return std::system_error(std::error_code(errno, std::generic_category()), what);
}

}
