#pragma once

namespace mon {

class Message;

class Client {
public:
    Client(const char *path);
    
    void send(const Message& msg) const;
    bool recv(Message& msg) const;
    bool send_then_recv(const Message& out, Message& in) const;
    
private:
    const char *path;
    
    int connect() const;
    void disconnect(int sock) const;
};

}
