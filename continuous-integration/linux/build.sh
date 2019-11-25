#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/../..")
BUILD_ROOT_DIR="$ROOT_DIR/build"

function build()
{
    COMPILER=$1
    BUILD_SUBDIR=$2
    BUILD_DIR="$BUILD_ROOT_DIR/$BUILD_SUBDIR"

    echo "-----------------------------------------------"
    echo "            Building with $COMPILER"
    echo "-----------------------------------------------"

    mkdir --parents "$BUILD_DIR" || exit $?
    cd "$BUILD_DIR" || exit $?
    cmake -DCMAKE_CXX_COMPILER="$COMPILER" \
        -DUSE_EIGEN3=OFF \
        -DUSE_OPENCV=OFF \
        -DWARNINGS=ON \
        -DWARNINGS_AS_ERRORS=OFF \
        ../.. || exit $?
    cmake --build . || exit $?
}

build clang++-8 clang || exit $?
build g++-7 gcc || exit $?
