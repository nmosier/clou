#include "pht.h"

// ----------------------------------------------------------------------------------------
// EXAMPLE 13:  Do the safety check into an inline function
//
// Comments: Output is unsafe.

static inline int is_x_safe(size_t x) { if (x < array1_size) return 1; return 0; }
void victim_function_v13(size_t x) {
  if (is_x_safe(x)) {
    atomic_thread_fence(memory_order_acquire);
    temp &= array2[array1[x] * 512];
  }
}

