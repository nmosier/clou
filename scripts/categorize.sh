#!/bin/bash

usage() {
    cat <<EOF
usage: $0 <dbg> <lkg>
EOF
}

if [[ $# -lt 2 ]]; then
    usage >&2
    exit 1
fi

DBG="$1"
shift 1

for LKG in "$@"; do
    for F in $(find "$LKG" -type f); do
	$(dirname "$0")/debuginfo.sh "$F" "$DBG" | head -1
    done
done
