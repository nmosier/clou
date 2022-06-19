#!/bin/bash

usage() {
    cat <<EOF
usage: $0 [-a] [-d <dir>] [-h] [<bench>...]
Options:
  -a        analyze results from all benchmarks
  -d <dir>  directory containing benchmark results (Clou output directories) [optional, default: $PWD]
  -h        show this help dialog
EOF
}

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"

BENCHES=()

DIR="$PWD"
while getopts "had:" OPTC; do
    case $OPTC in
	h)
	    usage
	    exit
	    ;;
	d)
	    DIR="$OPTARG"
	    ;;
	a)
	    BENCHES=(pht stl fwd new tea donna secretbox ssl3-digest mee-cbc)
	    ;;
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

if [[ "${#BENCHES[@]}" -gt 0 ]]; then
    if [[ $# -ne 0 ]]; then
	echo "$0: cannot combine '-a' (all benchmarks) and positional arguments" >&2
	exit 1
    fi
else
    if [[ $# -ne 0 ]]; then
	BENCHES="$@"
    else
	echo "$0: must specify either '-a' (all benchmarks) or provide positional arguments (individual benchmarks)" >&2
	exit 1
    fi
fi

# $1 -- benchmark name
get_types() {
    case "$1" in
	pht)
	    echo "v1"
	    ;;
	stl)
	    echo "v4"
	    ;;
	*)
	    echo "v1 v4"
	    ;;
    esac
}

# $1 -- benchmark name
get_xmits() {
    case "$1" in
	pht|stl|fwd|new)
	    echo "dt ct udt uct"
	    ;;
	openssl)
	    echo "udt"
	    ;;
	*)
	    echo "udt uct"
	    ;;
    esac
}

# <stdin> -- file to sum
get_runtime() {
    awk '
BEGIN {
  sum = 0;
} 
{
  sum += $0;
}
END {
  print sum;
}
'
}


for BENCH in "${BENCHES[@]}"; do
    TYPES="$(get_types "$BENCH")"
    XMITS="$(get_xmits "$BENCH")"
    for TYPE in $TYPES; do
	time_str=()
	bugs_str=()
	for XMIT in $XMITS; do
	    SUBDIR="$DIR/$BENCH-$TYPE-$XMIT-out"
	    if [[ -d "$SUBDIR" ]]; then
		# transmitters
		TOTAL_XMITS=$("${SCRIPT_DIR}/transmitters.sh" "$SUBDIR")
		PURE_XMITS=$("${SCRIPT_DIR}/addrs.sh" "$SUBDIR")
		bugs_str+=($(printf '%s(%s)' $TOTAL_XMITS $PURE_XMITS))

		# time
		RUNTIME=$(for F in $(find "$SUBDIR" -name "runtimes.txt"); do
		    cat $F
		done | get_runtime)
		time_str+=($RUNTIME)
	    else
		echo "warning: missing results directory \"$SUBDIR\"" >&2
		bugs_str+=("?(?)")
		time_str+=("?")
	    fi
	done

	# print results
	echo -n "$BENCH "

	case $TYPE in
	    v1)
		echo -n "Clou-PHT"
		;;
	    v4)
		echo -n "Clou-STL"
		;;
	    *)
		echo "$0: internal error: bad speculation type '$TYPE'" >&2
		exit 1
		;;
	esac

	echo -n " "
	echo -n "${time_str[@]}" | tr -s ' ' '/'
	echo -n " "
	echo -n "${bugs_str[@]}" | tr -s ' ' '/'
	echo

	
    done
done | column -t

	     
