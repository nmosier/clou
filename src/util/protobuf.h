#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <unistd.h>

#include "util/io.h"

namespace proto {

#if 0
template <class Message>
bool read(int fd, Message& msg) {
    uint32_t len;
    
    io::read(fd, len);
    std::vector<char> buf;
    io::read(fd, buf.data(), buf.size());
    return msg.ParseFromArray(buf.data(), buf.size());
}
#endif

template <class Message>
bool write(int fd, Message& msg) {
    std::string buf;
    if (!msg.SerializeToString(&buf)) {
        return false;
    }
    const uint32_t len = buf.size();
    io::write(fd, len);
    io::write(fd, buf.data(), buf.size());
    return true;
}

}
