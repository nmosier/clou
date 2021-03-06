#pragma once

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <sys/file.h>
#include <exception>
#include <gdbm.h>
#include <sstream>
#include <fcntl.h>
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

#if 0
namespace gdbm {

class File {
public:
    File() {}
    File(const std::string& path, int flags, int mode) {
        open(path, flags, mode);
    }
    
    ~File() {
        if (good()) {
            close();
        }
    }
    
    bool good() const { return f != nullptr; }
    operator bool() const { return good(); }
    
    void open(const std::string& path, int flags, int mode) {
        int oflags = 0;
        if ((flags & GDBM_WRCREAT)) {
            oflags |= O_RDWR | O_CREAT;
        } else if ((flags & GDBM_READER)) {
            oflags |= O_RDONLY;
        } else if ((flags & GDBM_WRITER)) {
            oflags |= O_RDONL
            
        

            int fd;
            if ((fd = ::open(path.c_str(), O_RDWR | O_CREAT, mode)) < 0) {
                throw std::system_error(errno, std::generic_category(), "open");
            }
            
            if ((f = ::gdbm_fd_open(fd, path.c_str(), 0, flags, nullptr)) == nullptr) {
                ::close(fd);
                throw gdbm_exception("gdbm_fd_open");
            }
        } else {
        }
    }
    
    void open(int fd, const std::string& path, int flags, int mode) {
        if ((f = ::gdbm_fd_open(fd, path.c_str(), 0, flags, mode, nullptr)) == nullptr) {
            throw gdbm_exception("gdbm_fd_open");
        }
    }
    
    void open(const std::string& path, int flags, int mode) {
        if ((f = ::gdbm_open(path.c_str(), 0, flags, mode, nullptr)) == nullptr) {
            throw gdbm_exception("gdbm_open");
        }
    }
    
    void close() {
        if (good()) {
            if (::gdbm_close(f) < 0) {
                throw gdbm_exception("gdbm_close");
            }
            f = nullptr;
        }
    }
    
    operator GDBM_FILE() const { return f; }
    
private:
    GDBM_FILE f = nullptr;
};

}
#endif

#if 0
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
#else
class SharedDatabase {
public:
    SharedDatabase() {}
    
    bool good() const { return path.empty(); }
    operator bool() const { return good(); }
    
    void open(const std::string& path) {
        this->path = path;
    }
    
    void close() {
        path.clear();
    }
    
    bool contains(const std::string& key) const {
        GDBM_FILE f = file();
        datum d = {
            .dptr = const_cast<char *>(key.data()),
            .dsize = static_cast<int>(key.size()),
        };
        const auto res = ::gdbm_exists(f, d);
        const auto err = gdbm_errno;
        ::gdbm_close(f);
        if (res) {
            return true;
        } else {
            if (err == GDBM_NO_ERROR) {
                return false;
            } else {
                throw gdbm_exception("gdbm_exists", err);
            }
        }
    }
    
    void insert(const std::string& key) {
        GDBM_FILE f = file();
        const datum k = {
            .dptr = const_cast<char *>(key.data()),
            .dsize = static_cast<int>(key.size()),
        };
        if (gdbm_store(f, k, k, 0) == -1) {
            ::gdbm_close(f);
            throw gdbm_exception("gdbm_store");
        }
        ::gdbm_close(f);
    }
    
private:
    std::string path;

    GDBM_FILE file() const {
        int fd = ::open(path.c_str(), O_RDWR | O_CREAT, 0664);
        if (fd < 0) {
            throw std::system_error(errno, std::generic_category(), "open");
        }
        if (::flock(fd, LOCK_EX) < 0) {
            throw std::system_error(errno, std::generic_category(), "flock");
        }
        
        GDBM_FILE f = ::gdbm_fd_open(fd, path.c_str(), 0, GDBM_WRCREAT, nullptr);
        if (f == nullptr) {
            ::close(fd);
            throw gdbm_exception("gdbm_open");
        }
        
        return f;
    }
};


#endif
