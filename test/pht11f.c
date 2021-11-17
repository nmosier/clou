#include "pht.h"
#include <string.h>
#include <stdint.h>

// ----------------------------------------------------------------------------------------
// EXAMPLE 11:  Use memcmp() to read the memory for the leak.
//
// Comments: Output is unsafe.

void victim_function_v11(size_t x) {
  if (x < array1_size) {
    atomic_thread_fence(memory_order_acquire);
    temp = memcmp(&temp, array2 + (array1[x] * 512), 1);
  }
}
