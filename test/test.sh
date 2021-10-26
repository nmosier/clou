#!/bin/bash

set -eu

usage() {
    cat <<EOF
usage: $0 [-h] -O <output-dir> -T <test-path>.c -R <reference-path> -L <lcm-path> -A <args>
EOF
}

ARGS="${ARGS-}"
CLANG='/opt/homebrew/opt/llvm@12/bin/clang-12'

while getopts "hO:T:R:L:A:C:" OPTC; do
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
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

mkdir -p "$OUTDIR"

LCM_ARGS="-o$OUTDIR $ARGS" "$CLANG" -fdeclspec -Wno-\#warnings -Xclang -load -Xclang "$LCM" -c "$TEST"

awk -F'--' '{print $2}' "$OUTDIR/leakage.txt" | sort > "$OUTDIR/leakage.txt.tmp"
sort "$REF" > "$OUTDIR/ref.txt"
echo "ACTUAL:"
cat "$OUTDIR/leakage.txt.tmp"
echo "REFERENCE:"
cat "$REF"
echo "COMM:"
comm -23 "$REF" "$OUTDIR/leakage.txt.tmp"
COMM=$(comm -23 "$OUTDIR/ref.txt" "$OUTDIR/leakage.txt.tmp")
! [[ "$COMM" ]]
