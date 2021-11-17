#include "pht.h"

// ----------------------------------------------------------------------------------------
// EXAMPLE 14:  Invert the low bits of x
//
// Comments: Output is unsafe.

void victim_function_v14(size_t x) {
  if (x < array1_size) {
    atomic_thread_fence(memory_order_acquire);
    temp &= array2[array1[x ^ 255] * 512];
  }
}
