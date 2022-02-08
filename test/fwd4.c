#include "fwd.h"

void example_4() {
    // attacker can use this to write secret data into benignIndex
    wrgadget_sec_for();

    // leak the same way as in Example 2, just with a different write gadget
    temp &= publicarray2[benignIndex * 512];
}
