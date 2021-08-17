#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <getopt.h>
#include <vector>

#include <llvm/Support/raw_ostream.h>

#include "config.h"

/* TODO
 * [ ] Handle function names
 */

static char prog[] = "lcm";
static std::vector<char *> args = {prog};

std::string output_dir;
unsigned verbose = 0;

static void usage(FILE *f = stderr) {
   const char *s = R"=(
usage: [option...]
Options:
  --help, -h           show help
  --output, -o <path>  output directory
  --func, -f <name>[,<name>]...  
                       only examine given functions
  --verbose, -v        verbosity++
)=";
   fprintf(f, s);
}

static int parse_args() {
   if (char *line = getenv("LCM_ARGS")) {
      while (char *s = strsep(&line, " ")) {
         if (*s) {
            args.push_back(s);
         }
      }
   }

   char **argv = args.data();
   int argc = args.size();
   int optc;

   struct option opts[] = {
      {"help", no_argument, nullptr, 'h'},
      {"verbose", no_argument, nullptr, 'v'},
      {"output", required_argument, nullptr, 'o'},
      {nullptr, 0, nullptr, 0}
   };
   
   while ((optc = getopt_long(argc, argv, "hvo:", opts, nullptr)) >= 0) {
      switch (optc) {
      case 'h':
         usage(stdout);
         exit(0);

      case 'o':
         output_dir = optarg;
         break;
         
      case 'v':
         ++verbose;
         break;

      default:
         usage();
         exit(1); 
      }
   }

   return 0;
}

static const int parse_args_force = parse_args();
