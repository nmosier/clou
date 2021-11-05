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

namespace mon {

template <typename T>
void Client::write(const T *buf, std::size_t count) {
    static_assert(std::is_pod<T>());
    assert(good());
    if (std::fwrite(buf, sizeof(T), count, f) != count) {
        if (std::feof(f)) {
            std::cerr << "fwrite: unexpected EOF\n";
        } else {
            std::perror("fwrite");
        }
        disconnect();
    } else {
        std::fflush(f);
    }
}

void Client::connect(const char *path) {
    assert(!good());
    
    int sock;
    
    if ((sock = ::socket(PF_UNIX, SOCK_STREAM, 0)) < 0) {
        std::perror("socket");
        return;
    }
    
    struct sockaddr_un addr;
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (::connect(sock, reinterpret_cast<const struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::perror("connect");
        ::close(sock);
        return;
    }
    
    if ((f = ::fdopen(sock, "r+")) == nullptr) {
        std::perror("fdopen");
        ::close(sock);
        return;
    }
}

void Client::disconnect() {
    assert(good());
    
    if (std::fclose(f) < 0) {
        std::perror("fclose");
    }
    f = nullptr;
}

void Client::send_step(const std::string& step, const std::string& func) {
    Message msg;
    FunctionStep *fs = msg.mutable_func_step();
    fs->set_step(step);
    fs->mutable_func()->set_name(func);
    send(msg);
}

void Client::send_connect() {
    ClientConnect msg;
    msg.set_pid(::getpid());
    send(msg);
}

}
