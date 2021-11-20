#include "stl.h"

// Case 11: _insecure_
//
// Same as case_3 but masking load is made by a function call. Because returned
// value is pushed on the stack, it can also be bypassed. */
uint8_t case_11_load_value(uint32_t idx) {
  register uint32_t ridx;
  ridx = idx & (array_size - 1);
  uint8_t to_leak = publicarray[ridx];
  return to_leak;
}
void case_11(uint32_t idx) {
  uint8_t to_leak = case_11_load_value(idx);

  /* Access overwritten secret */
  temp &= publicarray2[to_leak * 512];  
}

/* sourced store:    ENTRY
 * bypassed store:   store i32 %8
 * speculative load: %9
 * transmitter:      %11
 */
