#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <utility>

const char *prog;



namespace {
int argc;
char **argv;

void perror_exit(const char *s) {
    fprintf(stderr, "%s: %s: %s\n", prog, s, std::strerror(errno));
    std::exit(EXIT_FAILURE);
}

template <typename... Args>
void error(const char *fmt, Args&&... args) {
    fprintf(stderr, "%s: ", prog);
    fprintf(stderr, fmt, std::forward<Args>(args)...);
    fprintf(stderr, "\n");
    std::exit(EXIT_FAILURE);
}

}

void usage(FILE *f) {
    const char *s = R"=(usage: %s [-h] <fifo_path>
)=";
    ;
    fprintf(f, s, prog);
}

char *nextarg() {
    if (optind == argc) {
        usage(stderr);
        std::exit(EXIT_FAILURE);
    }
    return argv[optind++];
}

int main(int argc, char *argv[]) {
    ::argc = argc;
    ::argv = argv;
    ::prog = argv[0];
    
    const char *optstr = "h";
    int optchar;
    while ((optchar = getopt(argc, argv, optstr)) >= 0) {
        switch (optchar) {
            case 'h':
                usage(stdout);
                return EXIT_SUCCESS;
                
            default:
                usage(stderr);
                return EXIT_FAILURE;
        }
    }
    
    const char *path = nextarg();
    
    FILE *f;
    if ((f = std::fopen(path, "r+")) == nullptr) {
        perror_exit("fopen");
    }

    
    if (std::setvbuf(f, nullptr, _IONBF, 0) == EOF) {
        error("setvbuf: failed");
    }
    
    
    std::fclose(f);
}
