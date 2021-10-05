#include "pht.h"
#include <string.h>
#include <stdint.h>

// ----------------------------------------------------------------------------------------
// EXAMPLE 11:  Use memcmp() to read the memory for the leak.
//
// Comments: Output is unsafe.

int memcmp2(const void *s1_, const void *s2_, size_t n) {
  const uint8_t *s1 = (const uint8_t *) s1_;
  const uint8_t *s2 = (const uint8_t *) s2_;
  while ((*s1 || *s2) && *s1 == *s2) {
    ++s1; ++s2;
  }
  return (int) *s1 - (int) *s2;
}

void victim_function_v11(size_t x) {
     if (x < array1_size)
          temp = memcmp2(&temp, array2 + (array1[x] * 512), 1);
}
