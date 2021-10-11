#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR=$(realpath "$SCRIPT_DIR/../..")
SOURCE_DIR="$ROOT_DIR/source"

cppFiles=$(find "$SOURCE_DIR" -name '*.cpp')
hFiles=$(find "$SOURCE_DIR" -name '*.h' -not -path "$SOURCE_DIR/3rd-party/*")

if [ -z "$cppFiles" ]; then
    echo Error: Cannot find C++ source files
    exit 1
fi

echo "Checking formatting"
errorsFound=0
for fileName in $cppFiles $hFiles; do
    echo $fileName
    if diff $fileName <(clang-format-10 $fileName); then
        ((errorsFound = errorsFound + 1))
    else
        echo "ok"
    fi
done
echo
if [ $errorsFound -ne 0 ]; then
    echo "ERROR: $errorsFound formatting error(s) found! See the diffs for each file printed above."
    exit 1
fi

BUILD_DIR="$ROOT_DIR/build/ci/tidy"
mkdir --parents "$BUILD_DIR" || exit $?
cd "$BUILD_DIR" || exit $?
cmake -GNinja \
    -DCMAKE_CXX_CLANG_TIDY="/usr/bin/clang-tidy-10" \
    -DWARNINGS=ON \
    -DWARNINGS_AS_ERRORS=ON \
    -DEIGEN3_INCLUDE_DIR="/usr/include/eigen3" \
    "$SOURCE_DIR" || exit $?
cmake --build . || exit $?

echo "All files are properly formatted!" ["$0"]
