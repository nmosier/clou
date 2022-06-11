#!/bin/bash

UNIQLC="$(dirname ${BASH_SOURCE[0]})/uniqlc"

for LKG in $(find "$@" -type f); do
    head -50 "$LKG" | grep '^[[:digit:]]\+: \(LOAD\|STORE\)' | tail -1 | tr -s ' ' | cut -d' ' -f3-
done | "$UNIQLC"
