#!/bin/bash

if [[ $# -ne 1 ]]; then
    cat <<EOF
usage: $0 <leakage.txt>
EOF
    exit 1
fi

awk '
{
  if ($0 ~ /:[[:space:]]*$/) {
    printf "%s", $0;
  } else {
    print $0;
  }
}
' "$1" | grep -v -e '^$' -e ':[[:space:]]*$' -e '^[[:digit:]]' | wc -l
