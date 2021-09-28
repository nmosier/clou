#include <string>
#include <system_error>

#include "shm.h"

namespace {
std::system_error syserr(const std::string& what = "") {
    return std::system_error(std::error_code(errno, std::generic_category()), what);
}
}

void shared_memory::open(std::size_t size, int prot) {
    if (good()) {
        close();
    }
    size_ = size;
    if ((data_ = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANON, -1, 0)) == MAP_FAILED) {
        throw syserr("mmap");
    }
}

void shared_memory::close() {
    assert(good());
    if (munmap(data_, size_) < 0) {
        throw syserr("munmap");
    }
    data_ = nullptr;
    size_ = 0;
}

shared_memory::~shared_memory() {
    if (good()) {
        close();
    }
}

void *shared_memory_manager::allocate(std::size_t bytes) {
    if (next + bytes > mem->size()) {
        throw std::bad_alloc();
    }
    void *res = static_cast<char *>(mem->data()) + next;
    next += bytes;
    return res;
}
