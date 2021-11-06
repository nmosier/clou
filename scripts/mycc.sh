#!/bin/bash                                                                                                                                                                                                                                                                     

CLANG=clang-12
LLVM_DIS=llvm-dis-12

$CLANG "$@" # OBJ                                                                                                                                                                                                                                                               

# find -c flag                                                                                                                                                                                                                                                                  
cflag() {
    echo "$@" | awk '                                                                                                                                                                                                                                                           
{                                                                                                                                                                                                                                                                               
  for (i = 1; i <= NF; ++i) {                                                                                                                                                                                                                                                   
    if ($i == "-c") {                                                                                                                                                                                                                                                           
      print 1;                                                                                                                                                                                                                                                                  
    }                                                                                                                                                                                                                                                                           
  }                                                                                                                                                                                                                                                                             
}                                                                                                                                                                                                                                                                               
'
}

# find -o flag                                                                                                                                                                                                                                                                  
obj() {
    echo "$@" | awk '                                                                                                                                                                                                                                                           
{                                                                                                                                                                                                                                                                               
  for (i = 1; i <= NF; ++i) {                                                                                                                                                                                                                                                   
    if ($i == "-o") {                                                                                                                                                                                                                                                           
      print $(i+1);                                                                                                                                                                                                                                                             
    }                                                                                                                                                                                                                                                                           
  }                                                                                                                                                                                                                                                                             
}                                                                                                                                                                                                                                                                               
'
}

OBJ=$(obj "$@")
CFLAG=$(cflag "$@")

if ! [[ "$OBJ" ]] || ! [[ "$CFLAG" ]]; then
    exit
fi

BASE="${OBJ%.*}"
BC="${BASE}.bc"
LL="${BASE}.ll"

$CLANG "$@" -emit-llvm -o "$BC"
$LLVM_DIS < "$BC" > "$LL"
