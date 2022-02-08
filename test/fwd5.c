#include "fwd.h"

void example_5(uint64_t idx, uint8_t val, uint64_t idx2) {
    // E.g., this mask is 0xf if publicarray_size is 16.
    // 'volatile' ensures it's actually in memory.
    volatile uint64_t publicarray_size_mask = publicarray_size - 1;

    // attacker can use this to overwrite publicarray_size_mask
    wrgadget_2(idx, val);

    // leak the same way as in Example 1, just with a different write gadget
    temp &= publicarray2[publicarray[idx2 & publicarray_size_mask] * 512];
}

