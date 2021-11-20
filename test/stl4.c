#include "stl.h"

// Case 4: _insecure_
//
// Similar to case_1 but without intermediate pointers
void case_4(uint32_t idx) {
  register uint32_t ridx;
  ridx = idx & (array_size - 1);

  /* Overwrite secret value */
  secretarray[ridx] = 0;        // Bypassed store
  
  /* Access overwritten secret */
  temp &= publicarray2[secretarray[ridx] * 512];  
}
