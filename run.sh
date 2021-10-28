#!/bin/bash

set -e

usage() {
    cat <<EOF
usage: $0 [-hg] arg...
EOF
}

JOBS=`nproc`
ARGS=

CLANG=${LLVM_DIR}/bin/clang

OUTDIR="out"
LIBLCM="src/liblcm.so"

while getopts "hgx:j:oO:L:" OPTCHAR; do
    case "$OPTCHAR" in
	h)
	    usage
	    exit
	    ;;
	g)
	    DEBUGGER="lldb --"
	    ;;
	x)
	    ARGS+="$OPTARG "
	    ;;
	j)
	    JOBS="$OPTARG"
	    ;;
	o)
	    OPEN="yes"
	    ;;
	O)
	    OUTDIR="$OPTARG"
	    ;;
	L)
	    LIBLCM="$OPTARG"
	    ;;
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

mkdir -p "$OUTDIR"

ARGS+=" -j$JOBS "

ASAN_VARS="DYLD_INSERT_LIBRARIES=/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/lib/clang/13.0.0/lib/darwin/libclang_rt.asan_osx_dynamic.dylib"

LCM_ARGS="-o$OUTDIR -vvv $ARGS" $DEBUGGER $CLANG -fdeclspec -Wno-\#warnings -Xclang -load -Xclang "$LIBLCM" -c "$@"


if [[ "$OPEN" ]]; then
    make2 -sj
    IDS=$(cut -f1 -d' ' out/leakage.txt)
    FILES=()
    for ID in $IDS; do
	FILES+=("out/leakage-$ID.png")
    done
    open "${FILES[@]}"
fi
