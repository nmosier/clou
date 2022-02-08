#include "fwd.h"

void example_1(uint64_t idx, uint8_t val, uint64_t idx2) {
    // E.g., this mask is 0xf if publicarray_size is 16.
    // 'volatile' ensures it's actually in memory.
    volatile uint64_t publicarray_size_mask = publicarray_size - 1;

    // attacker can use this to overwrite publicarray_size_mask
    wrgadget(idx, val);

    // non-speculatively, this code is safe due to the mask applied to idx2.
    // The mask (rather than traditional bounds check) also makes this safe from
    //   Spectre v1.
    // However, by overwriting the mask, the attacker can read OOB off of
    //   publicarray, then observe where in publicarray2 was accessed in order to
    //   leak the secret.
    temp &= publicarray2[publicarray[idx2 & publicarray_size_mask] * 512];
}
