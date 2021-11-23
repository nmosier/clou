#include <stdint.h>
#include <stddef.h>

extern uint8_t A[];
extern uint8_t B[];
extern uint8_t C[];
extern size_t C_size;
extern uint8_t temp;

void victim_psf(size_t idx) {
  if (idx < C_size) {
    C[0] = 64;
    temp &= B[A[C[idx] * idx]];
  }
}
