#pragma once

#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <type_traits>
#include <sys/poll.h>

namespace io {

/* writes all input or aborts */
template <typename T>
void write(int fd, const T *buf_, std::size_t nitems) {
    const char *buf = reinterpret_cast<const char *>(buf_);
    std::size_t rem = nitems * sizeof(T);
    while (rem > 0) {
        ssize_t bytes = ::write(fd, buf, rem);
        if (bytes < 0) {
            std::perror("write");
            std::abort();
        } else if (bytes == 0) {
            std::cerr << "write: unexpected EOF\n";
            std::abort();
        }
        std::cerr << "write " << bytes << " bytes\n";
        rem -= bytes;
        buf += bytes;
    }
}

template <typename T>
void write(int fd, const T& value) {
    static_assert(std::is_pod<T>());
    write(fd, &value, 1);
}

template <typename T>
void read(int fd, T *buf_, std::size_t nitems) {
    char *buf = reinterpret_cast<char *>(buf_);
    std::size_t rem = nitems * sizeof(T);
    while (rem > 0) {
        ssize_t bytes = ::read(fd, buf, rem);
        if (bytes < 0) {
            std::perror("read");
            std::abort();
        } else if (bytes == 0) {
            std::cerr << "read: unexpected EOF\n";
            std::abort();
        }
        std::cerr << "read " << bytes << " bytes\n";
        rem -= bytes;
        buf += bytes;
    }
}

template <typename T>
void read(int fd, T& value) {
    static_assert(std::is_pod<T>());
    read(fd, &value, 1);
}


// NOTE: doesn't abort.
inline ssize_t readall(int fd, std::vector<char>& vec) {
    while (true) {
        constexpr std::size_t bufsize = 4096;
        char buf[bufsize];
        const ssize_t bytes = ::read(fd, buf, bufsize);
        if (bytes < 0) {
            return -1;
        } else if (bytes == 0) {
            return vec.size();
        } else {
            std::cerr << "read " << bytes << " bytes\n";
            std::copy(buf, buf + bytes, std::back_inserter(vec));
        }
    }
}

inline void pipe(int fds[2]) {
    if (::pipe(fds) < 0) {
        std::perror("pipe");
        std::abort();
    }
}

inline bool can_read(int fd) {
    struct pollfd pfd = {.fd = fd, .events = POLLIN};
    const auto res = ::poll(&pfd, 1, 0);
    if (res < 0) {
        std::perror("poll");
        std::abort();
    } else if (res == 0) {
        return false;
    } else {
        return pfd.revents & POLLIN;
    }
}

inline bool is_closed(int fd) {
    struct pollfd pfd = {.fd = fd, .events = POLLIN | POLLOUT};
    if (::poll(&pfd, 1, 0) < 0) {
        std::perror("poll");
        std::abort();
    }
    return pfd.revents & (POLLHUP | POLLERR);
}

}
