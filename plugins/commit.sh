#!/bin/bash
bld=$(tput bold)       # Bold
blu=$(tput setaf 4)    # Blue
rst=$(tput sgr0)       # Text reset

msg=$1

if [[ ! $msg ]]; then
	echo "no message"
	exit
fi

for f in *; do
  if [[ -d $f ]]; then
    cd $f
    if [[ -d .git ]]; then
      echo "${bld}${blu}committing $f${rst}"
      git commit -am "$msg"
    fi
    cd - 1> /dev/null
  fi
done
