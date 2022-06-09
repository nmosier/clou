#!/bin/bash

usage() {
    cat <<EOF
usage: $0 <lkg_file> <src_dir>
EOF
}

if [[ $# -ne 2 ]]; then
    usage >&2
    exit 1
fi

process_file() {

    # extract all paths in file
    grep '^[[:digit:]]\+: [^:]*: [^:]*:[[:digit:]]\+:$' "$1" | while read LINE; do
	F=$(echo $LINE | cut -d':' -f3 | tr -d ' ')
	L=$(echo $LINE | cut -d':' -f4 | tr -d ' ')
	tail -n+$L $2/$F | head -1
    done
    
}

process_file "$1" "$2"
