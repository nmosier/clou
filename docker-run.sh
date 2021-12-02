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
GUEST_ROOT="/lcm"


docker run -it --privileged --cap-add=SYS_PTRACE --security-opt=seccomp=unconfined \
       -v "${HOST_ROOT}/src":"${GUEST_ROOT}/src":ro \
       -v "${HOST_ROOT}/test":"${GUEST_ROOT}/test":ro \
       -v "${HOST_ROOT}/docs":"${GUEST_ROOT}/docs":ro \
       -v "${HOST_ROOT}/scripts":"${GUEST_ROOT}/scripts":ro \
       "$NAME"
