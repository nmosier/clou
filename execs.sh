#!/bin/bash

find out/ -name 'exec*.dot' -print0 | xargs -0 -P$(sysctl -n hw.ncpu) dot -Tpdf -O
