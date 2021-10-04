#include <stdint.h>
#include <stddef.h>

#define SIZE 16                 /* Size fo secretarray and publicarray */
uint32_t array_size = 16;

// Public values
uint8_t publicarray[SIZE] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
uint8_t publicarray2[512 * 256] = { 20 };

// The attacker's goal in all of these examples is to learn any of the secret data in secretarray
uint8_t secretarray[SIZE] = { 10,21,32,43,54,65,76,87,98,109,110,121,132,143,154,165 };

// This is mostly used to prevent the compiler from optimizing out certain operations
volatile uint8_t temp = 0;
