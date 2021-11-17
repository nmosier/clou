#include "pht.h"

// ----------------------------------------------------------------------------------------
// EXAMPLE 9:  Use a separate value to communicate the safety check status.
//
// Comments: Output is unsafe.

void victim_function_v09(size_t x, int *x_is_safe) {
  if (*x_is_safe) {
    atomic_thread_fence(memory_order_acquire);
    temp &= array2[array1[x] * 512];
  }
}
