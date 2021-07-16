#include <stdlib.h>

#define ARRLEN 8

int g_val;

int *f(unsigned idx) {
   int secret = 0xbaddecaf;
   int arr[ARRLEN];
   if (idx < ARRLEN) {
      // p can be assigned &secret in misspeculative execution.
      return &arr[idx];
   }
   return &g_val;
}
