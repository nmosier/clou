#include <sys/stat.h>
#include <libgen.h>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <system_error>
#include <string>

#include "output.h"

namespace util {

void mkdir(const std::string& path) {
    for (std::string::size_type pos = 0; pos != std::string::npos; ++pos) {
        pos = path.find("/", pos);
        const std::string subpath = path.substr(0, pos);
        if (subpath.empty()) { continue; }
        if (::access(subpath.c_str(), F_OK) < 0) {
            if (errno == ENOENT) {
                if (::mkdir(subpath.c_str(), 0774)) {
                    std::stringstream ss;
                    ss << "mkdir(" << subpath << ")";
                    throw std::system_error(errno, std::generic_category(), ss.str());
                }
            } else {
                throw std::system_error(errno, std::generic_category(), "access");
            }
        }
    }
}

std::string dirname(const std::string& path) {
    char *path_ = ::strdup(path.c_str());
    char *dir;
    if ((dir = ::dirname(path_)) == nullptr) {
        throw std::system_error(errno, std::generic_category(), "dirname");
    }
    std::free(path_);
    return dir;
}

}
