#!/bin/bash

set -e

usage() {
    cat <<EOF
usage: $0 [-h] -O <output-dir> -T <test-path>.c -R <reference-path> -L <lcm-path> -A <args> [-C <clang>] [-g] [-V]
EOF
}

CLANG="$(which clang-12)"
DEBUGGER=
VALGRIND=

if [ -z "$SRC" ]; then
    export SRC="$(dirname "$0")/.."
fi

while getopts "hO:T:R:L:A:C:gV" OPTC; do
    case $OPTC in
	h)
	    usage
	    exit
	    ;;
	O)
	    OUTDIR="$OPTARG"
	    ;;
	T)
	    TEST="$OPTARG"
	    ;;
	R)
	    REF="$OPTARG"
	    ;;
	L)
	    LCM="$OPTARG"
	    ;;
	A)
	    ARGS="$ARGS $OPTARG"
	    ;;
	C)
	    CLANG="$OPTARG"
	    ;;
	g)
	    DEBUGGER="lldb -- "
	    ;;
	V)
	    VALGRIND="valgrind -- "
	    ;;
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

rm -rf "$OUTDIR"
mkdir -p "$OUTDIR"

LCM_ARGS="-o $OUTDIR $ARGS" $DEBUGGER $VALGRIND "$CLANG" -g -fdeclspec $CFLAGS -Wno-\#warnings -Xclang -load -Xclang "$LCM" -c -emit-llvm -S -o "${OBJ}.o" "$TEST"

if ! [[ "$REF" && -f "$REF" ]]; then
    REF="/dev/null"
fi


diff <(sort "$REF") <(sort "$OUTDIR/test.out")
