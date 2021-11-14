#!/bin/bash

LCM=${LCM-"$PWD/../lcm"}
BUILD_TYPE=${BUILD_TYPE-Debug}
CC=${CC-$(which clang-12)}
CFLAGS=${CFLAGS-"-Wno-cpp -Xclang -load -Xclang $LCM/build/src/$BUILD_TYPE/liblcm.so"}
./configure CC="$CC" CFLAGS="$CFLAGS" LDFLAGS="" --disable-asm "$@"
