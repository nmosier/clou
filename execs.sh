#!/bin/bash

for DOT in out/exec*.dot; do
    dot -Tpdf -O $DOT
done
