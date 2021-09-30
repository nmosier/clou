#include "stl.h"

// Case 2: _insecure_
//
// Array access protected by index masking: this test case is insecure because
// the masked index is stored on the stack and therefore can be bypassed
void case_2(uint32_t idx) {
  idx = idx & (array_size - 1);

  // Bypassed store is not visible at source level

  /* Access overwritten secret */
  temp &= publicarray2[publicarray[idx] * 512];  
}
