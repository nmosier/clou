// #include "../../../__libsym__/sym.h"
#include "donna.c"


u8 mypublic[32];  // public
u8 secret[32];    // secret
u8 basepoint[32]; // public

int main() {

  curve25519_donna(mypublic,secret,basepoint);

  return 0;
}
