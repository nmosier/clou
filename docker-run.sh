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

HOST_ROOT="$PWD"
GUEST_ROOT="/clou"


docker run -it --privileged --cap-add=SYS_PTRACE --security-opt=seccomp=unconfined \
       -v "${HOST_ROOT}/src":"${GUEST_ROOT}/src" \
       -v "${HOST_ROOT}/test":"${GUEST_ROOT}/test" \
       -v "${HOST_ROOT}/docs":"${GUEST_ROOT}/docs" \
       -v "${HOST_ROOT}/scripts":"${GUEST_ROOT}/scripts" \
       "$NAME"
