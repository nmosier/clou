#!/bin/bash

set -u
set -e

usage() {
    cat <<EOF
usage: $0 -t v1|v4 -x <udt|uct|dt|ct> [-O <outdir>]
       $0 -h
Options:
  -h            show this help description
  -t v4         speculation primitive. Spectre v4 (STL) only relevant for these litmus tests.
  -x <xmitter>  transmitter type to look for [required]: udt, uct, dt, ct.
  -O <outdir>   output directory [optional, default: $PWD/stl-<xmitter>-out]
EOF
}

BENCH=fwd
SPECTRE_TYPE=
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
	    ;;
    esac
done

shift $((OPTIND-1))
if [[ $# -ne 0 ]]; then
    echo "$0: too many positional arguments" >&2
    exit 1
fi

if [[ -z "$SPECTRE_TYPE" ]]; then
    echo "$0: missing '-t'" >&2
    usage >&2
    exit 1
fi

if [[ -z "$TRANSMITTER" ]]; then
    echo "$0: missing '-x'"
    usage >&2
    exit 1
fi

case "$TRANSMITTER" in
	udt)
	    PATTERN="addr,addr"
	    ;;
	uct)
	    PATTERN="addr,ctrl"
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
CLOU_OUT="${CLOU_OUT-$PWD/$BENCH-$SPECTRE_TYPE-$TRANSMITTER-out}"
rm -rf $CLOU_OUT
mkdir $CLOU_OUT

export CLOU_OUT
. /clou/scripts/LCM_ARGS

case "$SPECTRE_TYPE" in
    v1)
	LCM_ARGS="$LCM_ARGS_V1"
	;;
    v4)
	LCM_ARGS="$LCM_ARGS_V4"
	;;
    *)
	echo "$0: -t: invalid Spectre type '$SPECTRE_TYPE'" >&2
	exit 1
	;;
esac

LCM_ARGS+=" --traceback=2 -fexample_.*"
export LCM_ARGS

for I in {1..5}; do
    NAME="$BENCH$I"
    SRC="/clou/test/$NAME.c"

    OUTDIR="$CLOU_OUT/$NAME"
    rm -rf "$OUTDIR"
    mkdir "$OUTDIR"

    export LCM_EXTRA_ARGS="-o $OUTDIR"
    clang-12 -g -Xclang -load -Xclang /clou/build/src/libclou.so -c "$SRC" -o "/clou/build/test/$NAME.o"
done
