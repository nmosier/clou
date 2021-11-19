#pragma once

#include <cstdio>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>

namespace semutil {

inline bool acquire(int& semid) {
    struct sembuf sop = {
        .sem_num = 0,
        .sem_op = -1,
        .sem_flg = SEM_UNDO
    };
    while (true) {
        if (::semop(semid, &sop, 1) < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                // repeat
            } else {
                std::perror("semop");
                semid = -1;
                return false;
            }
        } else {
            return true;
        }
    }
}

inline bool release(int& semid) {
    struct sembuf sop = {
        .sem_num = 0,
        .sem_op = 1,
        .sem_flg = SEM_UNDO
    };
    if (::semop(semid, &sop, 1) < 0) {
        std::perror("semop");
        semid = -1;
        return false;
    }
    return true;
}

}
