#!/bin/bash

usage() {
    cat <<EOF
usage: $0 <src_dir> [<dbg>]
EOF
}


if [[ $# -ne 2 ]]; then
    usage >&2
    exit 1
fi

SRC="$1"
DBG="$2"

mkdir -p yes no unk dbg/{yes,no,unk}

LINES=$(($(tput lines) / 2))

ID=0

inputs() {
    find "$SRC" -name '*.txt'
}

NUM_INPUTS=$(inputs | wc -l)

for FILE in $(find "$SRC" -name '*.txt'); do
    ((++ID))
    
    head -40 "$FILE"

    DBGTMP=$(mktemp)
    ~/lcm/scripts/debuginfo.sh $FILE $DBG > $DBGTMP
    FILTERED=
    for ANS in yes no unk; do
	for REF in $(find dbg/$ANS -type f); do
	    if diff -q "$DBGTMP" "$REF" >/dev/null; then
		OUT=$(mktemp $ANS/XXXXXXX)
		mv $FILE $OUT
		FILTERED=1
		break
	    fi
	done
	if [[ "$FILTERED" ]]; then
	    echo filtered "$FILE"
	    break
	fi	
    done
    if [[ "$FILTERED" ]]; then
	continue
    fi
    cat "$DBGTMP"

    awk "END { printf \"%.1f\\n\", $ID / $NUM_INPUTS * 100 }" /dev/null

    while true; do
	read ANS
	unset RES
	case "$ANS" in
	    y*)
		RES=yes
		;;
	    n*)
		RES=no
		;;
	    u*)
		RES=unk
		;;
	    *)
		;;
	esac

	if [[ "$RES" ]]; then
	    break;
	fi
    done

    OUT=$(mktemp "$RES/XXXXXX")
    mv "$FILE" "$OUT"

    cp $DBGTMP dbg/$RES

done
