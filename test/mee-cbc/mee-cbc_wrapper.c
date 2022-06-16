/* Compile with:
     clang -Imac-then-encrypt \
           mac-then-encrypt/aes128.c mac-then-encrypt/aes128cbc.c \
           mac-then-encrypt/sha256blocks.c mac-then-encrypt/hmac.c mac-then-encrypt/verify_32.c \
           mac-then-encrypt/pad128.c mac-then-encrypt/pad_cbc_aes128.c \
           mac-then-encrypt/mac_then_encrypt.c \
           mee-cbc.c
*/

#define LEN 64

#include "crypto_block.h"

unsigned char out[LEN];             // secret
unsigned long out_len;              // public
const unsigned char in[128];        // public
unsigned long in_len = LEN;         // public
const unsigned char iv[INPUTBYTES]; // public
const unsigned char enc_sk[16];     // secret
const unsigned char mac_sk[32];     // secret

extern int decrypt_then_verify(unsigned char *out,unsigned long *out_len, const unsigned char *in,unsigned long in_len,
			       const unsigned char *iv,const unsigned char *enc_sk,const unsigned char *mac_sk);

int main(){
  return decrypt_then_verify(out,&out_len,in,in_len,iv,enc_sk,mac_sk);
}
