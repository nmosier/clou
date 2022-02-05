#include "pht.h"

// ----------------------------------------------------------------------------------------
// EXAMPLE 5:  Use x as the initial value in a for() loop.
//
// Comments: Output is unsafe.

void victim_function_v05(size_t x) {
  size_t i;
  if (x < array1_size) {
    for (i = x - 1; i > 0; i--) {
      temp &= array2[array1[i] * 512];
    }
  }
}
