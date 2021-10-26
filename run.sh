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

while getopts "hgx:j:o" OPTCHAR; do
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
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

ARGS+=" -j$JOBS "

LCM_ARGS="-oout -vvv $ARGS" $DEBUGGER $CLANG -fdeclspec -Wno-\#warnings -Xclang -load -Xclang src/liblcm.so -c "$@"

if [[ "$OPEN" ]]; then
    make2 -sj
    IDS=$(cut -f1 -d' ' out/leakage.txt)
    FILES=()
    for ID in $IDS; do
	FILES+=("out/leakage-$ID.png")
    done
    open "${FILES[@]}"
fi
