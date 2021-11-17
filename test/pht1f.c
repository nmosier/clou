#include "pht.h"

void victim_function_v01(size_t x) {
  if (x < array1_size) {
    atomic_thread_fence(memory_order_acquire);
    temp &= array2[array1[x] * 512];
  }
  temp = 0;
}
