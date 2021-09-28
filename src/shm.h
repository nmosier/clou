#pragma once

#include <cstddef>
#include <sys/mman.h>

class shared_memory {
public:
    shared_memory() {}
    shared_memory(std::size_t size) { open(size); }
    shared_memory(const shared_memory&) = delete;
    shared_memory(shared_memory&&) = delete;
    
    std::size_t size() const { return size_; }
    void *data() const { return data_; }
    
    void open(std::size_t size, int prot = PROT_READ | PROT_WRITE);
    void close();
    
    bool good() const { return data_ != MAP_FAILED; }
    operator bool() const { return good(); }
    
    ~shared_memory();
    
private:
    std::size_t size_ = 0;
    void *data_ = MAP_FAILED;
};

class shared_memory_manager;

template <typename T>
class shared_memory_allocator {
public:
    using value_type = T;
    
    shared_memory_allocator(shared_memory_manager& manager): manager(manager) {}
    
    T *allocate(std::size_t n);
    void deallocate(T *p, std::size_t n);
    
private:
    shared_memory_manager& manager;
};

class shared_memory_manager {
public:
    shared_memory_manager(): mem(nullptr) {}
    shared_memory_manager(shared_memory& mem): mem(&mem) {}
    
    bool good() const { return mem != nullptr; }
    operator bool() const { return good(); }
    
    void *allocate(std::size_t bytes);
    void deallocate(void *p, std::size_t bytes) {}
    
private:
    shared_memory *mem;
    std::size_t next;
};


template <typename T>
T *shared_memory_allocator<T>::allocate(std::size_t n) {
    return static_cast<T *>(manager.allocate(sizeof(T) * n));
}

template <typename T>
void shared_memory_allocator<T>::deallocate(T *p, std::size_t n) {
    manager.deallocate(p, n * sizeof(T));
}
