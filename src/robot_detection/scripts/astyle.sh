#!/bin/bash
# @brief reformat all files in a folder recursivle, apply it to /src folders
# @author buck

if [[ $1 == "" ]]; then
  echo "usage: $0 <path-to-format>"
  exit
fi

find $1 -regextype posix-extended -regex '.*\.(cpp|h|cc|c|hpp)' | xargs \
  astyle --brackets=linux --align-pointer=type --unpad-paren
find $1 -name *.orig | xargs rm
