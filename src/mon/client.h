#pragma once

namespace mon {

class Message;

class Client {
public:
    Client(const char *path);
    
    void send(const Message& msg) const;
    void recv(Message& msg) const;
    void send_recv(const Message& out, Message& in) const;
    
private:
    int fd;
    
    void lock() const;
    void unlock() const;
    void send_locked(const Message& msg) const;
    void recv_locked(Message& msg) const;
};

}
