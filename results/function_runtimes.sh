#!/bin/bash

set -eu

usage() {
cat <<EOF
usage: $0 <directory or file>
EOF
}

if [[ $# -ne 1 ]]; then
    usage >&2
    exit 1
fi

DIR="$1"
FUNCS=$(find "$1" -name '*.[[:digit:]]*.log'  | xargs basename -a | cut -d'.' -f1 | sort | uniq)

calculate() {
    if ! [[ "$@" ]]; then
	echo calculate: bad args: "$@" >&2
	exit 1
    fi
    awk "BEGIN { print $@ }" < /dev/null
}

parse_time() {
    ACC=$(grep -o '[[:digit:]\.]\+' <<< "$1")
    SCALE=1
    case "$1" in
	*ms*)
	    SCALE=0.001
	    ;;
    esac
    calculate "$ACC * $SCALE"
}

# $1 -- filename
check_should_analyze() {
    if [ -f "$1" -a ! -s "$1" ]; then
    	echo ERROR: empty file: "$1" >&2
    	# exit 1
    fi
    ! grep -q '^skipping analyzed function' $1
}

extract_leakage_runtime() {
    if ! grep -q 'RUNTIME:'; then
	# there were no transmitters
	echo 0
	return
    fi < $1

    # there were some transmitters
    grep -o 'RUNTIME:.*$' $1 | awk '{ acc += $4 } END { print acc }'
}

FLOAT='[[:digit:]]\+\(\.[[:digit:]]*\)?'
TIMESTAMP="\[${FLOAT}\(s\|ms\)\]"

extract_main_runtime() {
    grep -o '^ANALYZED.*$' $1 | cut -d' ' -f2 | head -1
}

extract_serial_runtime() {
    T1=$(extract_leakage_runtime $1)
    T2=$(extract_main_runtime $1)
    if [[ "$T1" ]] && [[ "$T2" ]]; then
	calculate "$T1 + $T2"
    else
	echo "ERROR: $1 leakage_runtime=$T1, main_runtime=$T2" >&2
	exit 1
    fi
}

extract_parallel_runtime() {
    T1=$(extract_leakage_runtime $1)
    T2=$(extract_main_runtime $1)
    if [ "$T1" -a "$T2" ]; then
	calculate "$T1 / 64 + $T2"
    else
	echo "ERROR $1 leakage_runtime=$T1, main_runtime=$T2" >&2
	exit 1
    fi
}

extract_cfg_nodes() {
    grep '^cfg-expanded: [[:digit:]]\+ nodes$' $@ | grep -o '[[:digit:]]\+'
}

handle_file() {
    FILE="$1"
    FUNC=$(basename $FILE .log | grep -o '^[^\.]*')
    if ! check_should_analyze $FILE; then return; fi
    SERIAL_RUNTIME=$(extract_serial_runtime $FILE)
    PARALLEL_RUNTIME=$(extract_parallel_runtime $FILE)
    NODES=$(extract_cfg_nodes $FILE)

    if [ "$SERIAL_RUNTIME" -a "$NODES" -a "$PARALLEL_RUNTIME" ]; then
	echo $FUNC $NODES $SERIAL_RUNTIME $PARALLEL_RUNTIME
    else
	echo "ERROR: $FUNC nodes=$NODES serial=$SERIAL_RUNTIME parallel=$PARALLEL_RUNTIME"
    fi
}

if [[ -d $1 ]]; then
    for FILE in $DIR/*.[[:digit:]]*.log; do
	handle_file $FILE
    done
elif [[ -f $1 ]]; then
    handle_file $1
else
    echo bad file >&2
    exit 1
fi
