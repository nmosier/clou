#include <fcntl.h>
#include <iostream>

#include "db.h"
#include "util/exception.h"

void FileLock::lock(int operation) const {
    if (::flock(fd, operation) < 0) {
        throw util::syserr();
    }
}

CFile::CFile(int fd, const char *mode) {
    if ((fd = ::dup(fd)) < 0) {
        throw util::syserr();
    }
    if ((f = ::fdopen(fd, mode)) == nullptr) {
        throw util::syserr();
    }
}

bool SharedDatabaseListSet::contains(const std::string& s) const {
    int fd;
    if ((fd = ::open(path.c_str(), O_RDONLY | O_CREAT, 0664)) < 0) {
        throw util::syserr();
    }
    if (::flock(fd, LOCK_SH) < 0) {
        throw util::syserr();
    }
    FILE *f;
    if ((f = ::fdopen(fd, "r")) == nullptr) {
        throw util::syserr();
    }

    bool res = false;
    while (true) {
        char *line = nullptr;
        std::size_t size_ = 0;
        const ssize_t size = ::getline(&line, &size_, f);
        if (size < 0) {
            break;
        }
        
        if (size >= 1) {
            char& end = line[size - 1];
            if (end == '\n') {
                end = '\0';
            }
        }
        
        if (std::strcmp(line, s.c_str()) == 0) {
            res = true;
        }
        
        free(line);
        
        if (res) {
            break;
        }
    }
    
    if (std::ferror(f)) {
        std::cerr << strerror(errno) << "\n";
        throw util::syserr();
    }
    
    if (std::fclose(f) < 0) {
        throw util::syserr();
    }
    
    return res;
}

void SharedDatabaseListSet::insert(const std::string& s) const {
    int fd;
    if ((fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0664)) < 0) {
        throw util::syserr();
    }
    if (::flock(fd, LOCK_EX) < 0) {
        throw util::syserr();
    }
    FILE *f;
    if ((f = ::fdopen(fd, "a")) == nullptr) {
        throw util::syserr();
    }
    
    fprintf(f, "%s\n", s.c_str());
    
    if (std::fclose(f) < 0) {
        throw util::syserr();
    }
}
