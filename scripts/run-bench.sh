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
  -x <xmitter>   transmitter type to look for [optional, default=udt]:
     		   - udt: universal data transmitters    -> sets pattern "addr_gep,addr"
		   - uct: unviersal control transmitters -> sets pattern "addr_gep,ctrl"
		   - dt:  data transmitters 		 -> sets pattern "addr"
		   - ct:  control transmitters 		 -> sets pattern "ctrl"
  -p <pattern>   transmitter pattern to look for [optional, default: set by transmitter type "-x"]
     		   General form, as regexp: ((addr|addr_gep|ctrl),)*(addr|addr_gep|ctrl)
  -j <jobs>      number of parallel compilation jobs [optional, default: 1]
  -J <jobs>      max number of threads across all compilation jobs [optional, default: 1]
  -T <file-to>   file timeout, in seconds [optional, default: 3600]
  -O <out-dir>   output directory [optional, default: \$PWD/<benchmark>-<xmitter>-out]
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
TRANSMITTER=udt
while getopts "ht:p:j:J:T:O:x:" OPTC; do
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
	  x)
	      TRANSMITTER="$OPTARG"
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
    usage >&2
    exit 1
fi

if [[ $# -lt 1 ]]; then
    echo "$0: missing positional argument <benchmark>" >&2
    usage >&2
    exit 1
fi

# set pattern
if [[ -z "${PATTERN+a}" ]]; then
    case "$TRANSMITTER" in
	udt)
	    PATTERN="addr_gep,addr"
	    ;;
	uct)
	    PATTERN="addr_gep,ctrl"
	    ;;
	dt)
	    PATTERN="addr"
	    ;;
	ct)
	    PATTERN="ctrl"
	    ;;
	*)
	    echo "$0: invalid transmitter type \"$TRANSMITTER\"" >&2
	    exit 1
	    ;;
    esac
fi

BENCH="$1"
shift 1

# $1 -- spectre type
# returns LCM_ARGS as string
get_lcm_args() {
    # special overrides
    EXTRA=""
    if [[ "$BENCH" = "donna" && "$1" = "v4" ]]; then
	EXTRA="--window=350"
    elif [[ "$BENCH" = "libsodium" && "$1" = "v4" ]]; then
	if [[ "$TRANSMITTER" = "udt" ]]; then
	    EXTRA="--window=350"
	elif [[ "$TRANSMITTER" = "uct" ]]; then
	    EXTRA="--window=250 -d200"
	fi
    fi
    
    case "$1" in
	v1)
	    echo "$LCM_ARGS_V1 $EXTRA"
	    ;;
	v4)
	    echo "$LCM_ARGS_V4 $EXTRA"
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
CLOU_OUT="${CLOU_OUT-$PWD/$BENCH-$TRANSMITTER-out}"
rm -rf $CLOU_OUT
mkdir $CLOU_OUT

export CLOU_OUT
export THREADS
. "${LCM_ARGS_FILE}"
export LCM_ARGS="$(get_lcm_args "$SPECTRE_TYPE") -o $CLOU_OUT --file-timeout=$FILE_TIMEOUT --deps=$PATTERN $@"

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
