// ----------------------------------------------------------------------------------------
// Define the types used, and specify as extern's the arrays, etc. we will access.
// Note that temp is used so that operations aren't optimized away.
//
// Compilation flags:  cl /c /d2guardspecload /O2 /Faout.asm
// Note: Per Microsoft's blog post, /d2guardspecload flag will be renamed /Qspectre
//
// This code is free under the MIT license (https://opensource.org/licenses/MIT), but
// is intentionally insecure so is only intended for testing purposes.

#include <stdlib.h>
#include <stdint.h>
#include <stdatomic.h>

extern size_t array1_size, array2_size, array_size_mask;
extern uint8_t array1[], array2[], temp;
