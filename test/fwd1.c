#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

/********************************************************************
Victim code.
********************************************************************/
extern uint8_t array2[];
unsigned int array1_size = 16;
uint8_t unused1[64];
uint64_t array1[160] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
volatile uint8_t *ptr = &array2[0]; // volatile to avoid optimizing out accesses
uint8_t unused2[64]; 
uint8_t array2[256 * 512];

char *secret = "The Magic Words are Squeamish Ossifrage.";

uint8_t temp = 0;  /* Used so compiler won't optimize out victim_function() */

void victim_function(size_t x, size_t secret_idx) {
  if (x < array1_size) {
    array1[x] += secret[secret_idx] * 512;
    
    for (unsigned i = 0; i < 4; ++i) {
      unused1[0] = 0;
    }
    
    *ptr;
  }
}
