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
        echo "${bld}${blu}merge $branch in $f${rst}"
        git merge ${branch}
      else
        echo "${bld}${blu}cannot merge $branch in $f, doesn't exist${rst}"
      fi
    fi
    cd - 1> /dev/null
  fi
done
