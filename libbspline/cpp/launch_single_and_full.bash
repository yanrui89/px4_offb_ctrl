#!/bin/bash

echo "CALC SINGLE POINT"
./build/3d_bspline_single \
data/cpx.txt \
data/cpy.txt \
data/cpz.txt \
data/posx.txt \
data/posy.txt \
data/posz.txt \
data/time.txt

echo "CALC PATH"
./build/3d_bspline \
data/cpx.txt \
data/cpy.txt \
data/cpz.txt \
data/posx.txt \
data/posy.txt \
data/posz.txt \
data/time.txt