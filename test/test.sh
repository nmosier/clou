#!/bin/bash

set -eu

usage() {
    cat <<EOF
usage: $0 [-h] -O <output-dir> -T <test-path>.c -R <reference-path> -L <lcm-path> -A <args>
EOF
}

ARGS="${ARGS-}"
CLANG="$(which clang-12)"
DEBUGGER=
VALGRIND=
CFLAGS=${CFLAGS-}

while getopts "hO:T:R:L:A:C:gV" OPTC; do
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
	R)
	    REF="$OPTARG"
	    ;;
	L)
	    LCM="$OPTARG"
	    ;;
	A)
	    ARGS="$ARGS $OPTARG"
	    ;;
	C)
	    CLANG="$OPTARG"
	    ;;
	g)
	    DEBUGGER="lldb -- "
	    ;;
	V)
	    VALGRIND="valgrind -- "
	    ;;
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

mkdir -p "$OUTDIR"
rm -f "$OUTDIR/functions.db"

OBJ="${OUTDIR}/$(basename "${TEST}" .c)"

LCM_ARGS="-o$OUTDIR $ARGS" $DEBUGGER $VALGRIND "$CLANG" -fdeclspec $CFLAGS -Wno-\#warnings -Xclang -load -Xclang "$LCM" -c -emit-llvm -S -o "${OBJ}.o" "$TEST"

preprocess() {
    grep -v '^$' | sed 's/, !dbg[^;]*//g' | sort | tr -s ' '
}

awk -F'--' '{print $2}' "$OUTDIR/leakage.txt" | preprocess | awk '
{
  for (i = 1; i <= NF; ++i) {
    if (!($i ~ /#[[:digit:]]+/)) {
      if (i > 1) {
        printf " ";
      }
      printf "%s", $i;
    }
  }
  printf "\n";
}
' > "$OUTDIR/leakage.txt.tmp"

if ! [[ -f "$REF" ]]; then
    REF="/dev/null"
fi

preprocess < "$REF" > "$OUTDIR/ref.txt"
echo "ACTUAL:"
cat "$OUTDIR/leakage.txt.tmp"
echo "REFERENCE:"
cat "$OUTDIR/ref.txt"
echo "COMM:"
comm -23 "$OUTDIR/ref.txt" "$OUTDIR/leakage.txt.tmp"
COMM=$(comm -23 "$OUTDIR/ref.txt" "$OUTDIR/leakage.txt.tmp")

if [[ -s "$OUTDIR/ref.txt" ]]; then
    ! [[ "$COMM" ]]
else
    ! [[ -s "$OUTDIR/leakage.txt.tmp" ]]
fi
