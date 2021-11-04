#pragma once

#include <cstdio>

namespace mon {

class Message;

class Client {
public:
    Client(): f(nullptr) {}
    Client(const char *path): f(nullptr) { connect(path); }
    Client(Client&& other): f(other.f) {
        other.f = nullptr;
    }
    
    Client& operator=(Client&& other) {
        f = other.f; other.f = nullptr;
        return *this;
    }
    
    ~Client() { if (good()) { disconnect(); } }
    
    void send(const Message& msg);
    bool recv(Message& msg);
    bool send_then_recv(const Message& out, Message& in);
    
    bool good() const { return f != nullptr; }
    operator bool() const { return good(); }
    
private:
    FILE *f;
    
    void connect(const char *path);
    void disconnect();
    
    template <typename T>
    void write(const T *buf, std::size_t count);
};

}
