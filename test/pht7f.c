#include "pht.h"

// ----------------------------------------------------------------------------------------
// EXAMPLE 7:  Compare against the last known-good value.
//
// Comments: Output is unsafe.

void victim_function_v07(size_t x) {
     static size_t last_x = 0;
     if (x == last_x) {
       atomic_thread_fence(memory_order_acquire);
       temp &= array2[array1[x] * 512];
     }
     if (x < array1_size) {
       last_x = x;
     }
}
