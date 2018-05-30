#!/bin/bash

COLOR_END="\033[0m"
COLOR_RED="\033[0;31m" 
COLOR_GREEN="\033[0;32m" 
STYLE='google'

FILES_TO_CHECK=$(git diff --name-only master | grep -E ".*\.(cpp|c|h|hpp)"$)

if [ -z "${FILES_TO_CHECK}" ]; then
  echo -e "No diff to master."
  exit 0
fi

for f in $FILES_TO_CHECK; do
  d=$(diff -u "$f" <(clang-format "$f" -style=${STYLE}))
  if [ -z "$d" ]; then
    echo -e "Code style check passed."
    exit 0
  else
    echo -e "Code style check failed, please run clang-format"
    echo "$d" |
        sed -e "s/\(^-.*$\)/`echo -e \"$COLOR_RED\1$COLOR_END\"`/" |
        sed -e "s/\(^+.*$\)/`echo -e \"$COLOR_GREEN\1$COLOR_END\"`/"
    exit 1
  fi
done