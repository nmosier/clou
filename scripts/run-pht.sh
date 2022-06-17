#!/bin/bash

set -u
set -e

usage() {
    cat <<EOF
usage: $0 [-t v1] -x <udt|uct|dt|ct> [-O <outdir>]
       $0 -h
Options:
  -h            show this help description
  -t v1         speculation primitive. Spectre v1 (PHT) only relevant for this litmus tests.
  -x <xmitter>  transmitter type to look for [required]: udt, uct, dt, ct.
  -O <outdir>   output directory [optional, default: $PWD/pht-<xmitter>-out]
EOF
}

BENCH=pht
SPECTRE_TYPE=v1
TRANSMITTER=
while getopts "ht:x:O:" OPTC; do
    case $OPTC in
	h)
	    usage
	    exit
	    ;;
	t)
	    SPECTRE_TYPE="$OPTARG"
	    ;;
	x)
	    TRANSMITTER="$OPTARG"
	    ;;
	O)
	    CLOU_OUT="$OPTARG"
	    ;;
	*)
	    usage >&2
	    exit 1
    esac
done

shift $((OPTIND-1))
if [[ $# -ne 0 ]]; then
    echo "$0: too many positional arguments" >&2
    usage >&2
    exit 1
fi

if [[ "$SPECTRE_TYPE" != "v1" ]]; then
    echo "$0: -t: invalid Spectre type '$SPECTRE_TYPE'" >&2
    exit 1
fi

# set pattern
case "$TRANSMITTER" in
	udt)
	    PATTERN="addr_gep,addr"
	    ;;
	uct)
	    PATTERN="addr_gep,ctrl"
	    ;;
	dt)
	    PATTERN="addr"
	    ;;
	ct)
	    PATTERN="ctrl"
	    ;;
	*)
	    echo "$0: invalid transmitter type \"$TRANSMITTER\"" >&2
	    exit 1
	    ;;
esac

LCM_ARGS_FILE=/clou/scripts/LCM_ARGS
CLOU_OUT="${CLOU_OUT-$PWD/$BENCH-$TRANSMITTER-out}"
rm -rf $CLOU_OUT
mkdir $CLOU_OUT

export CLOU_OUT
. "${LCM_ARGS_FILE}"
export LCM_ARGS="$LCM_ARGS_V1 --traceback=2 --deps=$PATTERN -fvictim_function_.*"

for I in {1..15}; do
    NAME="pht${I}"
    OUTDIR="$CLOU_OUT/$NAME"
    rm -rf "$OUTDIR"
    mkdir "$OUTDIR"
    LCM_EXTRA_ARGS="-o $OUTDIR" clang-12 -g -fdeclspec -Xclang -load -Xclang /clou/build/src/libclou.so -c "/clou/test/$NAME.c" -o "/clou/build/test/$NAME.o"
done
