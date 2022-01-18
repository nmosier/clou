#!/bin/bash

set -e

# check if compiling or loading

COMPILE_FLAG_FOUND=
LINK_FLAG_FOUND=
SHARED_FOUND=
COMPILE_FOUND=

ARGS=()

if [[ ! "$OPT_ARGS" ]]; then
    OPT_ARGS=("-load" "@clou_lib@" "-lcm")
fi

for ARG in "$@"; do
    case "$ARG" in
	"__COMPILE__")
	    COMPILE_FLAG_FOUND=1
	    ;;
	"__LINK__")
	    LINK_FLAG_FOUND=1
	    ;;
	"-dynamiclib"|"-shared")
	    SHARED_FOUND=1
	    ARGS+=("$ARG")
	    ;;
	"-c")
	    COMPILE_FOUND=1
	    ARGS+=("$ARG")
	    ;;
	*)
	    ARGS+=("$ARG")
	    ;;
    esac
done

locate() {
    which "$@" | head -1 | xargs basename
}

CLANG="$(locate clang clang-12)"

compile() {
    "$CLANG" "$@"
}

link() {
    
    LLVM_OBJS=()
    OTHER_ARGS=()

    for ARG in "$@"; do
	if [[ "$ARG" =~ \.o$ ]]; then
	    if file "$ARG" | grep -q LLVM; then
		LLVM_OBJS+=("$ARG")
		continue
	    fi
	fi
	OTHER_ARGS+=("$ARG")
    done

    LLVM_LINK="$(locate llvm-link llvm-link-12)"
    LLVM_OPT="$(locate opt llvm-opt opt-12 llvm-opt-12)"

    LLVM_LINK_OUT="$(mktemp)"
    "$LLVM_LINK" -o "$LLVM_LINK_OUT" "${LLVM_OBJS[@]}"

    LLVM_LINK_OUT1="$(mktemp)"
    "$LLVM_OPT" "${OPT_ARGS[@]}" -o "$LLVM_LINK_OUT1" "$LLVM_LINK_OUT"

    "$CLANG" "$LLVM_LINK_OUT1" "${OTHER_ARGS[@]}"
}

if [[ "$LINK_FLAG_FOUND" && "$COMPILE_FLAG_FOUND" ]]; then
    if [[ ! "$COMPILE_FOUND" && "$SHARED_FOUND" ]]; then
	link "${ARGS[@]}"
    else
	"$CLANG" "${ARGS[@]}"
    fi
elif [[ "$COMPILE_FLAG_FOUND" ]]; then
    compile "${ARGS[@]}"
elif [[ "$LINK_FLAG_FOUND" || "$SHARED_FOUND" ]]; then
    link "${ARGS[@]}"
else
    "$CLANG" "${ARGS[@]}"
fi
