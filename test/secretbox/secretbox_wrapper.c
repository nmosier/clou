// #include "../../../__libsym__/sym.h"
#include "libsodium/include/sodium.h"
// #include "/lcm/build/libsodium/src/libsodium/crypto_secretbox/crypto_secretbox.c"
// #include "/lcm/build/libsodium/src/libsodium/crypto_secretbox/xsalsa20poly1305/secretbox_xsalsa20poly1305.c"
// #include "/lcm/build/libsodium/src/libsodium/crypto_stream/crypto_stream.c"

#define KEY_LEN crypto_secretbox_KEYBYTES                    /* 32 bytes */
#define MSG_LEN 256
#define NONCE_LEN crypto_secretbox_NONCEBYTES                /* 24 */
#define CIPHERTEXT_LEN (crypto_secretbox_MACBYTES + MSG_LEN)

unsigned char c[CIPHERTEXT_LEN];   // public
const unsigned char m[MSG_LEN];    // secret
unsigned long long mlen = MSG_LEN; // public
const unsigned char n[NONCE_LEN];  // public
const unsigned char k[KEY_LEN];    // secret


int main(){

  /* Run encryption */
  crypto_secretbox(c, m, mlen, n, k);
  
  return 0;
}
