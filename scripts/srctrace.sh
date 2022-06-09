#!/bin/bash

usage() {
    cat <<EOF
usage: $0 <lkg-file>
EOF
}

if [[ $# -ne 1 ]]; then
    usage >&2
    exit 1
fi

F="$1"

awk '
BEGIN {
  seen = 0;
}
{
  if (seen) {
    print $0;
    seen = 0;
  }
}

/:[[:digit:]]+:$/ {
  seen = 1;
}
' "$F"
