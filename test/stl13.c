#include "stl.h"

uint8_t case_11_load_value(uint32_t idx) {
  register uint32_t ridx;
  ridx = idx & (array_size - 1);
  uint8_t to_leak = publicarray[ridx];
  return to_leak;
}


// Case 13: _secure_
//
// Same as 11 but result of function is forced in register so it cannot be
// bypassed
void case_13(uint32_t idx) {  // SECURE
  register uint8_t to_leak;
  to_leak = case_11_load_value(idx);

  /* Access overwritten secret */
  temp &= publicarray2[to_leak * 512];  
}

/* sourced store:    ENTRY
 * bypassed store:   store i32 %8
 * speculative load: %9
 * transmitter:      %11
 */
