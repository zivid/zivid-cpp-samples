#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR=$(realpath "$SCRIPT_DIR/../..")
SOURCE_DIR="$ROOT_DIR/source"

cppFiles=$(find "$SOURCE_DIR" -name '*.cpp')
hFiles=$(find "$SOURCE_DIR" -name '*.h')

if [ -z "$cppFiles" ]; then
    echo Error: Cannot find C++ source files
    exit 1
fi

for fileName in $cppFiles $hFiles; do
    echo $fileName
    diff $fileName \
        <(clang-format-8 $fileName) \
        || exit $?
done
echo Success! ["$0"]
