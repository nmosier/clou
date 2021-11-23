#!/bin/bash

set -eu

usage() {
    cat <<EOF
usage: $0 <name>
EOF
}

if [[ $# -ne 1 ]]; then
    usage >&2
    exit 1
fi

NAME="$1"

docker build --build-arg=build_type=RelWithDebInfo -t "${NAME}" .
