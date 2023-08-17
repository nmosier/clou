#!/bin/bash

set -u
set -e

usage() {
    cat <<EOF
usage: $0 [-t v4] -x <udt|uct|dt|ct> [-O <outdir>]
       $0 -h
Options:
  -h            show this help description
  -t v4         speculation primitive. Spectre v4 (STL) only relevant for these litmus tests.
  -x <xmitter>  transmitter type to look for [required]: udt, uct, dt, ct.
  -O <outdir>   output directory [optional, default: $PWD/stl-<xmitter>-out]
EOF
}

BENCH=stl
SPECTRE_TYPE=v4
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
    echo "$0: missing '-t'"
    usage >&2
    exit 1
elif [[ "$SPECTRE_TYPE" != "v4" ]]; then
    echo "$0: -t: invalid Spectre type '$SPECTRE_TYPE'" >&2
    exit 1
fi

if [[ -z "$TRANSMITTER" ]]; then
    echo "$0: missing '-x'"
    usage >&2
    exit 1
fi

# set pattern
# set pattern
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
. "${LCM_ARGS_FILE}"
export LCM_ARGS="$LCM_ARGS_V4 --traceback=2 --deps=$PATTERN"


# ./stl12r.ll
# ./stl6r.ll
# ./stl9_bisr.ll
# ./stl13r.ll
# ./stl9r.ll
# ./stl3r.ll
# ./stl4r.ll
# ./stl10r.ll
# ./stl11r.ll
# ./stl5r.ll

# STL_SRCS=(stl1.c stl2.c stl3r.ll stl4r.ll stl5r.ll stl6r.ll stl7.c stl8.c stl9r.ll stl9_bisr.ll stl10r.ll stl11r.ll stl12r.ll stl13r.ll)
# NOTE: We're running it on the unmodified code for now, since this is what we did in the paper.
STL_SRCS=(stl1.c stl2.c stl3.c stl4.c stl5.c stl6.c stl7.c stl8.c stl9.c stl9_bis.c stl10.c stl11.c stl12.c stl13.c)

for BASE_EXT in ${STL_SRCS[@]}; do
    STL="/clou/test/$BASE_EXT"
    BASE="$(basename "$(basename "$BASE_EXT" .c)" .ll)"
    NAME="$(basename "$BASE" r)"
    
    # compute extra args
    EXTRA=""
    case "$NAME" in
	stl11|stl13)
	    EXTRA="--traceback=3"
	    ;;
	*)
	    ;;
    esac

    OUTDIR="$CLOU_OUT/$NAME"
    rm -rf "$OUTDIR"
    mkdir "$OUTDIR"

    export LCM_EXTRA_ARGS="-o $OUTDIR $EXTRA -fcase_${NAME#stl}"
    
    case "$BASE_EXT" in
	*.c)
	    clang-12 -g -Xclang -load -Xclang /clou/build/src/libclou.so -c "$STL" -o "/clou/build/test/$BASE.o"
	    ;;
	*.ll)
	    opt-12 --load=/clou/build/src/libclou.so --lcm < "$STL" > "/clou/build/test/$BASE.o"
	    ;;
	*)
	    echo "$0: internal error: unexpected extension in '$STL'" >&2
	    exit 1
	    ;;
    esac
done
