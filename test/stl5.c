#include "stl.h"

// Case 5: _insecure_
//
// Overwrite private pointer with public pointer
uint8_t *case5_ptr = secretarray;
void case_5(uint32_t idx) {
  register uint32_t ridx;
  ridx = idx & (array_size - 1);

  case5_ptr = publicarray;      // Bypassed store

  uint8_t toleak = case5_ptr[ridx];
  temp &= publicarray2[toleak * 512];   
}
