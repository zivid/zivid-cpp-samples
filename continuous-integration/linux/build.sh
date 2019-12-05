#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/../..")
BUILD_ROOT_DIR="$ROOT_DIR/build/ci"

#There's a C++11 compatibility bug in the Zivid API which makes it fail on
#older versions of Clang, such as the default Clang 3.8.2 on Ubuntu 16.04.
#An internal Zivid issue to fix this has been logged. When that issue has
#been fixed, this entire option should be removed.
if [[ "$1" == "--skip-clang" ]]; then
    SKIP_CLANG=true
fi

#TODO 16.04 default pcl is too old. See issue #43
source /etc/os-release || exit $?
if [[ "$VERSION_ID" == "16.04" ]]; then
    OS_SPECIFIC_OPTIONS="-DUSE_PCL=OFF"
fi

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
    cmake -GNinja -DCMAKE_CXX_COMPILER="$COMPILER" \
        -DUSE_EIGEN3=OFF \
        -DUSE_OPENCV=OFF \
        -DWARNINGS=ON \
        -DWARNINGS_AS_ERRORS=ON \
        "$OS_SPECIFIC_OPTIONS" \
        ../../.. || exit $?
    cmake --build . || exit $?
}

if [[ ! "$SKIP_CLANG" ]]; then
    build clang++ clang || exit $?
fi
build g++ gcc || exit $?
