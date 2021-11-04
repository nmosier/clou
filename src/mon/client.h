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
  FILE *f;
    
    void connect(const char *path);
    void disconnect(int sock);
    
    template <typename T>
    void write(const T *buf, std::size_t count) const;
};

}
