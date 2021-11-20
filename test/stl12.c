#include "stl.h"

uint32_t case_10_do_mask(uint32_t idx) {
  register uint32_t ridx;
  ridx = idx & (array_size - 1);
  return ridx;
}


// Case 12: _secure_
//
// Same as 10 but result of function is forced in register so it cannot be
// bypassed
void case_12(uint32_t idx) {
  register uint32_t ridx;
  ridx = case_10_do_mask(idx);
  
  /* Access overwritten secret */
  temp &= publicarray2[publicarray[ridx] * 512];  
}

/* sourced store:    ENTRY
 * bypassed store:   store i32 %5
 * speculative load: %6
 * transmitter: %14
 */
