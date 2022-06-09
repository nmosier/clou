#!/bin/bash

usage() {
    cat <<EOF
usage: $0 <dir>...
EOF
}

if [[ $# -eq 0 ]]; then
    usage >&2
    exit 1
fi

process_file() {
    awk '
BEGIN {
  n = 0;
}
/(ADDR|ADDR_GEP|CTRL)/ {
  n = 1;
}
/^[[:digit:]]+: (LOAD|STORE)/ {
  if (n) {
    for (i = 3; i <= NF; ++i) {
      printf "%s ", $i;
      if (match($i, /:$/)) {
        break;
      }
    }
    print "";
  }  
  n = 0;
}
' "$1" | tail -1 # get last dependency
}

process_dir() {
    for FILE in $(find "$1" -type f); do
	process_file "$FILE"
    done
}

for DIR in "$@"; do
    process_dir "$DIR"
done
