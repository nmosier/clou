#!/bin/bash

usage() {
    cat <<EOF
usage: $0 [-hg] arg...
EOF
}

JOBS=`nproc`
ARGS=

while getopts "hgx:j:" OPTCHAR; do
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
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

ARGS+=" -j$JOBS "

LCM_ARGS="-oout -vvv $ARGS" $DEBUGGER clang-12 -fdeclspec -Wno-\#warnings -Xclang -load -Xclang src/liblcm.so -c "$@"
