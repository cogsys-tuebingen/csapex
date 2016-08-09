#!/bin/bash
bld=$(tput bold)       # Bold
blu=$(tput setaf 4)    # Blue
rst=$(tput sgr0)       # Text reset

branch=$1

if [[ ! $branch ]]; then
	echo "no branch specified"
	exit
fi

for f in *; do
  if [[ -d $f ]]; then
    cd $f
    if [[ -d .git ]]; then
      test=$(git branch | grep $branch)
      if [[ $test ]]; then
        echo "${bld}${blu}delete $branch in $f${rst}"
        git branch -d "$branch"
      fi
    fi
    cd - 1> /dev/null
  fi
done
