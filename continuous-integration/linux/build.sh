#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/../..")

cd $ROOT_DIR
mkdir build || exit $?
cd build || exit $?
cmake -DCMAKE_CXX_COMPILER=clang++-8 \
    -DUSE_EIGEN3=OFF \
    -DUSE_OPENCV=OFF \
    .. || exit $?
cmake --build . || exit $?
