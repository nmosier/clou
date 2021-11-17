#include "pht.h"

// ----------------------------------------------------------------------------------------
// EXAMPLE 8:  Use a ?: operator to check bounds.

void victim_function_v08(size_t x) {
  temp &= array2[array1[x < array1_size ?
			(atomic_thread_fence(memory_order_acquire), (x + 1)) :
			0]
		 * 512];
}
