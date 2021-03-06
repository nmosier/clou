#include "stl.h"

// Case 10: _insecure_
//
// Same as case_3 but masking is made by a function call. Because returned value
// is pushed on the stack, it can also be bypassed. */
uint32_t case_10_do_mask(uint32_t idx) {
  register uint32_t ridx;
  ridx = idx & (array_size - 1);
  return ridx;
}

void case_10(uint32_t idx) {
  uint32_t fidx = case_10_do_mask(idx);
  
  /* Access overwritten secret */
  temp &= publicarray2[publicarray[fidx] * 512];  
}

/* sourced store:    ENTRY
 * bypassed store:   store i32 %5...
 * speculative load: %6
 * transmitter:      %14
 */
