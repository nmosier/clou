#include "stl.h"

// Case 9: _secure_
//
// Same as case_4 but the store should not be bypassed assuming no speculation
// on conditionals. When the programs fetches the last line, the store that
// overwrites the secret should be retired because the sequence of instruction
// of the for loop is longer than the the reorder buffer.
void case_9(uint32_t idx) {
  uint32_t ridx;
  ridx = idx & (array_size - 1);

  /* Overwrite secret value */
  secretarray[ridx] = 0;        // *Not* bypassed store

  uint32_t i;
  for (i = 0; i < 200; ++i) temp &= i;
  
  /* Access overwritten secret */
  temp &= publicarray2[secretarray[ridx] * 512];  
}
