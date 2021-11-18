#!/bin/bash

set -eu

usage() {
    cat <<EOF
usage: $0 [-h] -O <output-dir> -T <test-path>.c -L <lcm-path> -A <args>
EOF
}

ARGS="${ARGS-}"
CLANG="$(which clang-12)"

while getopts "hO:T:L:A:" OPTC; do
    case $OPTC in
	h)
	    usage
	    exit
	    ;;
	O)
	    OUTDIR="$OPTARG"
	    ;;
	T)
	    TEST="$OPTARG"
	    ;;
	L)
	    LCM="$OPTARG"
	    ;;
	A)
	    ARGS="$ARGS $OPTARG"
	    ;;
	*)
	    usage >&2
	    exit 1
	    ;;
	esac
done

if ! [[ "$OUTDIR" && "$TEST" && "$LCM" && "$OPT" ]]; then
    echo "missing flags"
    exit 1
fi

shift $((OPTIND-1))

if [[ $# -ne 0 ]]; then
    echo "extra positional arguments"
    exit 1
fi

PASS_DIRS=("$OUTDIR/pass1" "$OUTDIR/pass2")
rm -rf "${PASS_DIRS[@]}"
mkdir -p "${PASS_DIRS[@]}"

OBJ_PREFIX="${OUTDIR}/$(basename "${TEST}" .c)"
LL="${OBJ_PREFIX}.ll"

export LCM_ARGS="-o${OUTDIR} ${ARGS}"

# Run first pass with fence insertion
"$CLANG" -fdeclspec -Wno-\#warnings -Xclang -load -Xclang "$LCM" -c -emit-llvm -S -o "${LL}" "$TEST"

# Run second pass to verify there's no leakage
"$OPT" -load "$LCM" -lcm < "${LL}" > "${OBJ_PREFIX}.bc"

# Check there's no leakage
if [[ -s "$OUTDIR/pass2/leakage.txt" ]]; then
    echo "FAIL: found leakage in repaired program"
    echo "leakage.txt:"
    cat "$OUTDIR/leakage.txt"
    exit 1
else
    echo "PASS"
fi
