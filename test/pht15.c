#include "pht.h"

// ----------------------------------------------------------------------------------------
// EXAMPLE 15:  Pass a pointer to the length
//
// Comments: Output is unsafe.

void victim_function_v15(size_t *x) {
     if (*x < array1_size)
          temp &= array2[array1[*x] * 512];
}
