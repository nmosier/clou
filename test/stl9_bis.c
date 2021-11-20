#include "stl.h"

// Case 9_bis: _insecure_
//
// Same as case_9 but this time the loop is too short. The store is not retired
// yet store is not retired yet when the load is executed and can therefore be
// bypassed.
void case_9_bis(uint32_t idx) {
  register uint32_t ridx;
  ridx = idx & (array_size - 1);

  /* Overwrite secret value */
  secretarray[ridx] = 0;        // Bypassed store

  register uint32_t i;
  for (i = 0; i < 10; ++i) temp &= i;
  
  /* Access overwritten secret */
  temp &= publicarray2[secretarray[ridx] * 512];  
}

/* sourced store:    ENTRY
 * bypassed store:   store i8 0
 * speculative load: %28
 * transmitter:      %33
 */
