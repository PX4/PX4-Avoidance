#!/bin/bash

COLOR_END="\033[0m"
COLOR_RED="\033[0;31m" 
COLOR_GREEN="\033[0;32m" 
STYLE='google'

FORMAT_DIFF=$(git diff -U0 HEAD^ | python ~/clang-format-diff.py -p1 -style=${STYLE})

if [ -z "${FORMAT_DIFF}" ]; then
  echo -e "Code style check passed."
  exit 0
else
  echo -e "Code style check failed, please run clang-format"
  echo "$FORMAT_DIFF" | 
  	sed -e "s/\(^-.*$\)/`echo -e \"$COLOR_RED\1$COLOR_END\"`/" |
  	sed -e "s/\(^+.*$\)/`echo -e \"$COLOR_GREEN\1$COLOR_END\"`/"
  exit 1
fi
