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
LL_IN="${OBJ_PREFIX}-in.ll"
LL_OUT="${OBJ_PREFIX}-out.ll"


# $1 -- pass number
get_dir() {
    DIR="${OUTDIR}/pass$1"
    mkdir -p $DIR
    echo $DIR
}

# $1 -- pass number
get_ll_in() {
    echo $(get_dir $1)/in.bc
}

# $1 -- pass number
get_ll_out() {
    echo $(get_dir $1)/out.bc
}

get_leakage() {
    echo $(get_dir $1)/leakage.txt
}

CFLAGS="-fdeclspec -Wno-#warnings"
LCMFLAGS="-Xclang -load -Xclang $LCM"

# Compile to LLVM IR with Clang
$CLANG $CFLAGS -c -emit-llvm -o $(get_ll_in 0) $TEST

# Iteratively run until there is no leakage

analyze() {
    env LCM_ARGS="${ARGS} -o$(get_dir $1)" $OPT -load $LCM -lcm < $(get_ll_in $1) > $(get_ll_out $1)
    [[ -f $(get_leakage $1) ]] && [[ -s $(get_leakage $1) ]]
}


I=0
while analyze $I; do
    cp $(get_ll_out $I) $(get_ll_in $((I+1)))
    ((I++))
done
echo "$I passes"

exit

# Run first pass with fence insertion
LCM_ARGS="${ARGS} -o${OUTDIR}/pass1" "$CLANG" -fdeclspec -Wno-\#warnings -Xclang -load -Xclang "$LCM" -c -emit-llvm -o "${LL}" "$TEST"

# Run second pass to verify there's no leakage
LCM_ARGS="${ARGS} -o${OUTDIR}/pass2" "$OPT" -load "$LCM" -lcm < "${LL}" > "${OBJ_PREFIX}.bc"

if ! [[ -f "$OUTDIR/pass2/leakage.txt" ]]; then
    echo "FAIL: no leakage.txt emitted for pass 2"
    exit 1
fi

# Check there's no leakage
if [[ -s "$OUTDIR/pass2/leakage.txt" ]]; then
    echo "FAIL: found leakage in repaired program"
    echo "leakage.txt:"
    cat "$OUTDIR/leakage.txt"
    exit 1
fi

echo PASS
