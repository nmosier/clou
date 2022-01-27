#!/bin/bash

set -e

usage() {
    cat <<EOF
usage: $0 [-X <arg>]... <name>
EOF
}

DOCKER_BUILD_ARGS=()

while getopts "hX:" optc; do
    case $optc in
	"h")
	    usage
	    exit
	    ;;
	"X")
	    DOCKER_BUILD_ARGS+=("$OPTARG")
	    ;;
	*)
	    usage >&2
	    exit 1
    esac
done

shift $((OPTIND-1))

if [[ $# -ne 1 ]]; then
    usage >&2
    exit 1
fi

NAME="$1"

docker build --build-arg=build_type=RelWithDebInfo "${DOCKER_BUILD_ARGS[@]}" -t "${NAME}" .
