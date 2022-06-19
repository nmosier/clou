#!/bin/bash

usage() {
    cat <<EOF
usage: $0 <lkg> <out>
EOF
}

if [[ $# -ne 2 ]]; then
    usage >&2
    exit 1
fi

DIR="$1"
OUT="$2"
mkdir -p "$OUT"

get_trace() {
    head -50 "$@" | grep '^[[:digit:]]\+: \(LOAD\|STORE\)' | tr -s ' ' | cut -d' ' -f3-
}

for LKG in $(find "$DIR" -type f); do
    LEN=$(head -50 "$LKG" | awk '/^(ADDR|ADDR_GEP|DATA|RF|CTRL)$/' | wc -l)
    if [[ $LEN -eq 2 ]]; then
	cp "$LKG" "$(mktemp "$OUT/XXXXXX.txt")"
    fi
done
