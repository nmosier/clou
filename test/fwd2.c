#include "fwd.h"

void example_2(uint64_t idx) {
    // attacker can use this to write secret data into benignIndex
    wrgadget_sec(idx);

    // non-speculatively, this code is safe because benignIndex is always
    //   in-bounds.
    // However, by writing secret data into benignIndex, the attacker can leak
    //   the secret data through the cache side channel (where in publicarray2
    //   was accessed)
    temp &= publicarray2[benignIndex * 512];
}
