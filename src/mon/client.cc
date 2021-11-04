#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>

#include "client.h"
#include "mon/proto.h"

namespace util {

inline std::system_error syserr(const std::string& what = "") {
    return std::system_error(std::error_code(errno, std::generic_category()), what);
}

}

namespace mon {

Client::Client(const char *path) {
    connect(path);
}

template <typename T>
void Client::write(const T *buf, std::size_t count) const {
    static_assert(std::is_pod<T>());
    if (std::fwrite(buf, sizeof(T), count, f) != count) {
        if (std::feof(f)) {
            std::cerr << "fwrite: unexpected EOF\n";
            std::exit(EXIT_FAILURE);
        } else {
            throw util::syserr("fwrite");
        }
    }
    std::fflush(f);
}

void Client::send(const Message& msg) const {
    std::string buf;
    msg.SerializeToString(&buf);
    const uint32_t buflen = htonl(buf.size());
    write(&buflen, 1);
    write(buf.data(), buf.size());
}

void Client::connect(const char *path) {
    int sock;
    
    if ((sock = ::socket(PF_UNIX, SOCK_STREAM, 0)) < 0) {
        throw util::syserr("socket");
    }
    
    struct sockaddr_un addr;
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (::connect(sock, reinterpret_cast<const struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        throw util::syserr("connect");
    }
    
    if ((f = ::fdopen(sock, "r+")) == nullptr) {
        throw util::syserr("fdopen");
    }
}

void Client::disconnect(int sock) {
    if (std::fclose(f) < 0) {
        throw util::syserr();
    }
    f = nullptr;
}

}
