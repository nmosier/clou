#include "fwd.h"

void example_3(uint64_t idx, uint8_t mask) {
    // attacker can use this to write secret data into benignIndex
    wrgadget_sec(idx);

    // non-speculatively, this code is safe because no secret data is processed.
    // However, by writing secret data into benignIndex, the attacker can
    //   learn any arbitrary bit of the secret data (by setting 'mask' appropriately)
    if (benignIndex & mask) {
        temp += temp;
    } else {
        temp += 2;
    }
}
