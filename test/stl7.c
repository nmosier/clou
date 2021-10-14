#include "stl.h"

// Case 7: _insecure_
//
// Same as case_2 but the mask is put in a variable
uint32_t case7_mask = UINT32_MAX;
void case_7(uint32_t idx) {
  case7_mask = (array_size - 1); // Bypassed store

  uint8_t toleak = publicarray[idx & case7_mask];
  temp &= publicarray2[toleak * 512];
}

/* sourced store:    ENTRY
 * bypassed store:   store i32 %5...
 * speculative load: %7
 * transmitter:      %17
 */
