#!/bin/bash

usage() {
    cat <<EOF
usage: $0 [-hg] arg...
EOF
}

while getopts "hg" OPTCHAR; do
    case "$OPTCHAR" in
	h)
	    usage
	    exit
	    ;;
	g)
	    DEBUGGER="lldb --"
	    ;;
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

LCM_ARGS="-oout -vvv" $DEBUGGER clang-12 -o /dev/null -Xclang -load -Xclang src/liblcm.so -c "$@"
