#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "client.h"
#include "mon/proto.h"

namespace util {

inline std::system_error syserr(const std::string& what = "") {
    return std::system_error(std::error_code(errno, std::generic_category()), what);
}

}

namespace mon {

Client::Client(const char *path): path(path) {}

void Client::send(const Message& msg) const {
    const int sock = connect();
    msg.SerializeToFileDescriptor(sock);
    disconnect(sock);
}

bool Client::recv(Message& msg) const {
    const int sock = connect();
    const bool res = msg.ParseFromFileDescriptor(sock);
    disconnect(sock);
    return res;
}

bool Client::send_then_recv(const Message& out, Message& in) const {
    const int sock = connect();
    out.SerializeToFileDescriptor(sock);
    const bool res = in.ParseFromFileDescriptor(sock);
    disconnect(sock);
    return res;
}

int Client::connect() const {
    int sock;
    
    if ((sock = ::socket(PF_UNIX, SOCK_STREAM, 0)) < 0) {
        throw util::syserr("socket");
    }
    
    struct sockaddr_un addr;
    addr.sun_family = AF_UNIX;
    ::strlcpy(addr.sun_path, path, sizeof(addr.sun_path));
    if (::connect(sock, reinterpret_cast<const struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        throw util::syserr("connect");
    }
    
    return sock;
}

void Client::disconnect(int sock) const {
    if (::close(sock) < 0) {
        throw util::syserr("close");
    }
}

}
