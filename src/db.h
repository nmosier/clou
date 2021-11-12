#pragma once

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <sys/file.h>

class FileLock {
public:
    FileLock(): fd(-1) {}
    FileLock(int fd, int operation): fd(fd) {
        lock(operation);
    }
    
    ~FileLock() {
        lock(LOCK_UN);
    }
    
private:
    int fd;
    
    void lock(int operation) const;
};

class CFile {
public:
    CFile(int fd, const char *mode);
    
    ~CFile() {
        ::fclose(f);
    }
    
    FILE *handle() const {
        return f;
    }
    
private:
    FILE *f;
};

class SharedFileLock: public FileLock {
public:
    SharedFileLock(int fd): FileLock(fd, LOCK_SH) {}
};



class SharedDatabaseListSet {
public:
    SharedDatabaseListSet() {}
    SharedDatabaseListSet(const std::string& path): path(path) {}
    bool contains(const std::string& s) const;
    void insert(const std::string& s) const;
    
private:
    std::string path;
};
