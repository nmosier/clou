#!/bin/bash

for LKG in $(find "$@" -type f); do
    head -50 "$LKG" | grep '^[[:digit:]]\+: \(LOAD\|STORE\)' | tail -1 | tr -s ' ' | cut -d' ' -f3-
done | uniqlc
