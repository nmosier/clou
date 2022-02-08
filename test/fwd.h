#include <stdint.h>

#include "clou.h"

// FORCEDINLINE definition from spectector-clang/13.c
#ifdef __MSVC__
#define FORCEDINLINE __forceinline
#else
#define FORCEDINLINE __attribute__((always_inline))
#endif

CLOU_ARCH_UNCONTROLLED uint64_t publicarray_size = 16;
CLOU_ARCH_UNCONTROLLED uint8_t publicarray[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
CLOU_ARCH_UNCONTROLLED uint8_t publicarray2[512 * 256] = { 20 };

// The attacker's goal in all of these examples is to learn any of the secret data in secretarray
CLOU_ARCH_UNCONTROLLED uint64_t secretarray_size = 16;
CLOU_ARCH_UNCONTROLLED uint8_t secretarray[16] = { 10,21,32,43,54,65,76,87,98,109,110,121,132,143,154,165 };

// 'volatile' ensures it's actually in memory
CLOU_ARCH_UNCONTROLLED volatile uint8_t benignIndex = 0;

// this is mostly used to prevent the compiler from optimizing out certain operations
CLOU_ARCH_UNCONTROLLED volatile uint8_t temp = 0;

// gadget that allows (speculatively) writing OOB off of publicarray,
//   so potentially overwriting any attacker-chosen 8-bit location with an attacker-chosen 'val'
FORCEDINLINE static void wrgadget(uint64_t idx, uint8_t val) {
    if (idx < publicarray_size) {
        publicarray[idx] = val;
    }
}

// gadget with the same behaviors as wrgadget(),
//   just formulated slightly differently
FORCEDINLINE static void wrgadget_2(uint64_t idx, uint8_t val) {
    // E.g., this mask is 0xf if publicarray_size is 16.
    const uint64_t publicarray_size_mask = publicarray_size - 1;

    if ((idx & publicarray_size_mask) == idx) {
        publicarray[idx] = val;
    }
}

// gadget that allows (speculatively) writing secret data
//   to an attacker-chosen location (OOB off of secretarray)
FORCEDINLINE static void wrgadget_sec(uint64_t idx) {
    if (idx < secretarray_size) {
        secretarray[idx] = secretarray[0];
    }
}

// gadget that allows (speculatively) writing secret data
//   to locations slightly off the end of secretarray
//   (by having the for loop perform additional iterations)
FORCEDINLINE static void wrgadget_sec_for() {
    for (unsigned i = 0; i < secretarray_size; i++) {
        secretarray[i] = secretarray[0];
    }
}
