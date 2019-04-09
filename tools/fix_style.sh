#!/bin/bash

STYLE="google"

if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <src_file | dir>"
    echo ""
    echo "ERROR: At least one source file or one directory must be provided!"

    exit 1
fi

for arg in "$@"
do
    if [ -f $arg ]; then
        clang-format-3.8 -i -style=${STYLE} $arg
    elif [ -d $arg ]; then
        find $arg -iname '*.h' -o -iname '*.cpp' | xargs clang-format-3.8 -i -style=${STYLE}
        find $arg -iname '*.h' -o -iname '*.cpp' | xargs chmod 644
    fi
done
