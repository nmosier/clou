#include "stl.h"

// Case 6: _insecure_
//
// Overwrite index to a table
uint32_t case6_idx = 0;
uint8_t *case6_array[2] = { secretarray, publicarray };
void case_6(uint32_t idx) {
  register uint32_t ridx;
  ridx = idx & (array_size - 1);

  case6_idx = 1;                // Bypassed store

  uint8_t toleak = (case6_array[case6_idx])[ridx];
  temp &= publicarray2[toleak * 512];
}

/* 
 * <ENTRY> -- sourced store
 * store i32 1, ... -- bypassed store
 * %9  -- speculative load
 * %22 -- transmitter
 */
