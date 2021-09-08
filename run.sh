#!/bin/bash

usage() {
    cat <<EOF
usage: $0 [-hg] arg...
EOF
}

ARGS=

while getopts "hgx:" OPTCHAR; do
    case "$OPTCHAR" in
	h)
	    usage
	    exit
	    ;;
	g)
	    DEBUGGER="lldb --"
	    ;;
	x)
	    ARGS+="$OPTARG"
	    ;;
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

LCM_ARGS="-oout -vvv $ARGS" $DEBUGGER clang-12 -o /dev/null -Xclang -load -Xclang src/liblcm.so -c "$@"
