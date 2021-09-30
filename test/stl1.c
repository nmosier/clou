#include "stl.h"

// Case 1: _insecure_
//
// Based on original POC for Spectre-v4,
// https://github.com/IAIK/transientfail/blob/master/pocs/spectre/STL/main.c
void case_1(uint32_t idx) {
  uint32_t ridx;
  ridx = idx & (array_size - 1);

  uint8_t* data = secretarray;
  uint8_t** data_slowptr = &data;
  uint8_t*** data_slowslowptr = &data_slowptr;
  
  /* Overwrite secret value */
  (*(*data_slowslowptr))[ridx] = 0; // Bypassed store
  
  /* Access overwritten secret */
  temp &= publicarray2[secretarray[ridx] * 512];  
}
