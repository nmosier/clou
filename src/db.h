#pragma once

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <sys/file.h>
#include <exception>
#include <gdbm.h>
#include <sstream>
#include <mutex>

class gdbm_exception: public std::exception {
public:
    gdbm_exception(const std::string& func, int err = gdbm_errno) {
        std::stringstream ss;
        ss << func << ": " << gdbm_strerror(err);
        msg = ss.str();
    }
    
    virtual const char *what() const noexcept override {
        return msg.c_str();
    }
    
private:
    std::string msg;
};

class FileMutex {
public:
    FileMutex(int fd, int operation): fd(fd), operation(operation) {}
    
    void lock() {
        lock(operation);
    }
    
    void unlock() {
        lock(LOCK_UN);
    }
    
private:
    int fd;
    int operation;
    
    void lock(int operation) {
        if (::flock(fd, operation) < 0) {
            throw std::system_error(errno, std::generic_category(), "flock");
        }
    }
};

class SharedDatabase {
public:
    SharedDatabase() {}
    SharedDatabase(const std::string& path) {
        open(path);
    }
    
    ~SharedDatabase() {
        if (good()) {
            close();
        }
    }
    
    /* Opening/Closing Operations */
    void open(const std::string& path) {
        if ((f = gdbm_open(path.c_str(), 0, GDBM_WRCREAT | GDBM_NOLOCK | GDBM_SYNC, 0664, nullptr)) == nullptr) {
            throw gdbm_exception("gdbm_open");
        }
    }
    
    void close() {
        gdbm_close(f);
        f = nullptr;
    }
    
    bool good() const { return f != nullptr; }
    operator bool() const { return good(); }
    
    /* Modifiers */
    
    bool contains(const std::string& key) const {
        datum d = {
            .dptr = const_cast<char *>(key.data()),
            .dsize = static_cast<int>(key.size()),
        };
        FileMutex m = mutex(LOCK_SH);
        std::unique_lock<FileMutex> l {m};
        const datum ent = gdbm_fetch(f, d);
        if (ent.dptr == nullptr) {
            if (gdbm_errno == GDBM_ITEM_NOT_FOUND) {
                return false;
            } else {
                throw gdbm_exception("gdbm_fetch");
            }
        } else {
            return true;
        }
    }
    
    void insert(const std::string& key) {
        const datum k = {
            .dptr = const_cast<char *>(key.data()),
            .dsize = static_cast<int>(key.size()),
        };
        FileMutex m = mutex(LOCK_EX);
        std::unique_lock<FileMutex> l {m};
        if (gdbm_store(f, k, k, 0) == -1) {
            throw gdbm_exception("gdbm_store");
        }
        if (gdbm_sync(f) < 0) {
            throw gdbm_exception("gdbm_sync");
        }
    }
    
private:
    GDBM_FILE f = nullptr;

    int fd() const {
        return gdbm_fdesc(f);
    }
    
    FileMutex mutex(int operation) const {
        return FileMutex(fd(), operation);
    }
};
