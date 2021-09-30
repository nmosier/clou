#include "stl.h"

// Case 8: _insecure_
//
// Index is multiplied by 0 to avoid overflows
uint32_t case8_mult = 200;
void case_8(uint32_t idx) {
  case8_mult = 0;               // Bypassed store

  uint8_t toleak = publicarray[idx * case8_mult];
  temp &= publicarray2[toleak * 512];
}
