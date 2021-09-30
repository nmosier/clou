#include "stl.h"

// Case 3: _secure_
//
// Same as case_2 but the index is forced into a register so the example is now
// secure
void case_3(uint32_t idx) {
  uint32_t ridx;
  ridx = idx & (array_size - 1);
  
  /* Access overwritten secret */
  temp &= publicarray2[publicarray[ridx] * 512];  
}
// In the following examples the index is put in a register so masking is not
// bypassed
