#!/bin/bash

set -u
set -e

usage() {
    cat <<EOF
usage: $0 -t v1|v4 [-p <pattern>=addr_gep,addr] [-j=1] [-J=1] [-h] [--] <benchmark> [<clou_arg>...]
Options:
  -h		 show this help description
  -t v1|v4       type of Spectre leakage to detect [required]:
     	    	   - v1: Spectre v1 leakage
		   - v4: Spectre v4 leakage
  -p <pattern>   transmitter pattern to look for [optional, default: "addr_gep,addr"]:
     		   General form, as regexp: ((addr|addr_gep|ctrl),)*(addr|addr_gep|ctrl)
		   Commonly used patterns:
     		   - addr_gep,addr -- subset of universal data leakage that feature array accesses
		   - addr,addr     -- all universal data leakage
		   - ctrl,addr     -- universal control leakage
		   - addr  	   -- plain data leakage
		   - ctrl	   -- plain control leakage
  -j <jobs>      number of parallel compilation jobs [optional, default: 1]
  -J <jobs>      max number of threads across all compilation jobs [optional, default: 1]
  -T <file-to>   file timeout, in seconds [optional, default: 3600]
  -O <out-dir>   output directory [optional, default: \$PWD/clou-out]
Positional arguments: 
  <benchmark>    The benchmark to run. This can be one of: tea, donna, secretbox, ssl3-digest, mee-cbc.
  <clou_arg>...  Additional configuration parameters to pass to Clou. These will be added
                 to the LCM_ARGS environment variable. To get a full list of these, use the Clou argument 
		 '--help' like so: $0 -t v1 -- --help
EOF
}

JOBS=1
THREADS=1
SPECTRE_TYPE=
FILE_TIMEOUT=3600
while getopts "ht:p:j:J:T:O:" OPTC; do
      case $OPTC in
	  h)
	      usage
	      exit
	      ;;
	  t)
	      SPECTRE_TYPE="$OPTARG"
	      ;;
	  p)
	      PATTERN="$OPTARG"
	      ;;
	  j)
	      JOBS="$OPTARG"
	      ;;
	  J)
	      THREADS="$OPTARG"
	      ;;
	  T)
	      FILE_TIMEOUT="$OPTARG"
	      ;;
	  O)
	      CLOU_OUT="$OPTARG"
	      ;;
	  *)
	      usage >&2
	      exit 1
	      ;;
      esac
done

shift $((OPTIND-1))

# check arguments
if [[ -z "${SPECTRE_TYPE}" ]]; then
    echo "$0: -t: required" >&2
    exit 1
fi

if [[ $# -lt 1 ]]; then
    echo "$0: missing positional argument <benchmark>" >&2
    exit 1
fi

BENCH="$1"
shift 1

# $1 -- spectre type
# returns LCM_ARGS as string
get_lcm_args() {
    case "$1" in
	v1)
	    echo "$LCM_ARGS_V1"
	    ;;
	v4)
	    echo "$LCM_ARGS_V4"
	    ;;
	*)
	    echo "$0: -t: invalid type" >&2
	    exit 1
	    ;;
    esac
}

# $1 -- benchmark name
# returns path
get_bench_path() {
    case "$1" in
	libsodium)
	    echo "/clou/build/libsodium"
	    ;;
	openssl)
	    echo "/clou/build/openssl"
	    ;;
	*)
	    echo "/clou/test/$1"
	    ;;
    esac
}

LCM_ARGS_FILE=/clou/scripts/LCM_ARGS

BENCH_PATH=$(get_bench_path "$BENCH")
cd "$BENCH_PATH"
CLOU_OUT="${CLOU_OUT-$PWD/clou-out}"
rm -rf $CLOU_OUT
mkdir $CLOU_OUT

export CLOU_OUT
export THREADS
. "${LCM_ARGS_FILE}"
export LCM_ARGS="$(get_lcm_args "$SPECTRE_TYPE") -o $CLOU_OUT --file-timeout=$FILE_TIMEOUT $@"

WIN="$BENCH"
MONITOR=monitor
BUILD=build
SOURCES=(*.c)
tmux new -s $WIN -d
tmux setenv -t $WIN CLOU_OUT $CLOU_OUT
tmux select-pane -T $MONITOR
tmux send-keys -t $WIN "/clou/build/src/mon/mon -f $CLOU_OUT/fifo -j$THREADS" C-m
tmux split-window -v -t $WIN
tmux select-pane -T $BUILD
tmux send-keys -t $WIN "make -j$JOBS" C-m
tmux attach -t $WIN
