#!/bin/bash

usage() {
    cat <<EOF
usage: $0 [-h] [-f <field>=0] -s <step> [-d <delim>=' ']
EOF
}

FIELD=0
DELIM=' '

while getopts "hf:s:d:" OPTC; do
    case $OPTC in
	h)
	    usage
	    exit
	    ;;
	f)
	    FIELD="$OPTARG"
	    ;;
	s)
	    STEP="$OPTARG"
	    ;;
	d)
	    DELIM="$OPTARG"
	    ;;
	*)
	    usage >&2
	    exit 1
	    ;;
    esac
done

shift $((OPTIND-1))

if [[ $# -ne 0 ]]; then
    usage >&2
    exit 1
fi

awk -vFS="$DELIM" "
BEGIN {
  max = 0;
  print \"nodes\", \"reps\";
}
{
  x = \$$FIELD;
  x_ = int(x / $STEP) * $STEP + $STEP / 2;
  if (x_ >= max) {
    max = x_;
  }
  data[x_]++;
}
END {
  for (x = $STEP / 2; x <= max; x += $STEP) {
    print x, int(data[x]);
  }
}
"
