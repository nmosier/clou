#include <fcntl.h>
#include <sys/file.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "client.h"
#include "mon/proto.h"

namespace util {

inline std::system_error syserr(const std::string& what = "") {
    return std::system_error(std::error_code(errno, std::generic_category()), what);
}

}

namespace mon {

Client::Client(const char *path) {
    if ((fd = ::open(path, O_RDWR)) < 0) {
        throw util::syserr();
    }
}

#if 0
void Client::lock() const {
    if (::flock(fd, LOCK_EX) < 0) {
        throw util::syserr();
    }
}

void Client::unlock() const {
    if (::flock(fd, LOCK_UN) < 0) {
        throw util::syserr();
    }
}
#elif 0
void Client::lock() const {
    if (::lockf(fd, F_LOCK, 0) < 0) {
        throw util::syserr();
    }
}

void Client::unlock() const {
    if (::lockf(fd, F_ULOCK, 0) < 0) {
        throw util::syserr();
    }
}
#else
void Client::lock() const {}
void Client::unlock() const {}
#endif

void Client::send(const Message& msg) const {
    lock();
    send_locked(msg);
    unlock();
}

void Client::recv(Message& msg) const {
    lock();
    recv_locked(msg);
    unlock();
}

void Client::send_recv(const Message& out, Message& in) const {
    lock();
    send_locked(out);
    recv_locked(in);
    unlock();
}

void Client::send_locked(const Message& msg) const {
    msg.SerializeToFileDescriptor(fd);
}

void Client::recv_locked(Message& msg) const {
    msg.ParseFromFileDescriptor(fd);
}

}
