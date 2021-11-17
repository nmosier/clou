#include "pht.h"

// ----------------------------------------------------------------------------------------
// EXAMPLE 6:  Check the bounds with an AND mask, rather than "<".
//
// Comments: Output is unsafe.

void victim_function_v06(size_t x) {
  if ((x & array_size_mask) == x) {
    atomic_thread_fence(memory_order_acquire);
    temp &= array2[array1[x] * 512];
  }
}
