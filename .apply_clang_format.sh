#!/bin/sh

maj_min=1
maj_max=8

base=clang-format
format=""

#Redirect output to stderr.
exec 1>&2

#check if clang - format is installed
type "$base" >/dev/null 2>&1 && format="$base"

#if not, check all possible versions
#(i.e.clang - format - <$maj_min - $maj_max> - <0 - 9>)
if [ -z "$format" ]
then
    for j in `seq $maj_max -1 $maj_min`
    do
        for i in `seq 0 9`
        do
            type "$base-$j.$i" >/dev/null 2>&1 && format="$base-$j.$i" && break
        done
        [ -z "$format" ] || break
    done
fi

#no versions of clang - format are installed
if [ -z "$format" ]
then
    echo "$base is not installed. Pre-commit hook will not be executed."
    exit 0
fi

#do the formatting
for file in `find -name *.cpp -or -name *.h -or -name *.hpp`
do
    echo $file
    "$format" -i "$file"
done
