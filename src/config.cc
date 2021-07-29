#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <getopt.h>
#include <vector>

#include <llvm/Support/raw_ostream.h>

#include "config.h"

static char prog[] = "lcm";
static std::vector<char *> args = {prog};

std::string cfg_output_path;
std::string aegpo_output_path;

static void usage(FILE *f = stderr) {
   const char *s = R"=(
usage: [option...]
Options:
  --help, -h      show help
  --cfg <path>    output path to full po CFG
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

   enum Option {
      CFG = 256,
      AEGPO,
   };
   
   struct option opts[] = {
      {"help", no_argument, nullptr, 'h'},
      {"cfg", required_argument, nullptr, CFG},
      {"aegpo", required_argument, nullptr, AEGPO},
      {0},
   };
   
   while ((optc = getopt_long(argc, argv, "h", opts, nullptr)) >= 0) {
      switch (optc) {
      case 'h':
         usage(stdout);
         exit(0);

      case CFG:
         cfg_output_path = optarg;
         break;

      case AEGPO:
         aegpo_output_path = optarg;
         break;

      default:
         usage();
         exit(1); 
      }
   }

   return 0;
}

static const int parse_args_force = parse_args();
