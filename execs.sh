#!/bin/bash

find out/ -name 'exec*.dot' -print0 | xargs -0 dot -Tpdf -O
