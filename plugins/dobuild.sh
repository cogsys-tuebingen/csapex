#!/bin/bash
bld=$(tput bold)       # Bold
blu=$(tput setaf 4)    # Blue
rst=$(tput sgr0)       # Text reset

packages=$(find . -iname 'package.xml' | sed -s 's|/package.xml||')
for f in $packages; do
  if [[ -d $f ]]; then
    cd $f
    echo "${bld}${blu}enabling $f${rst}"
    rm CATKIN_IGNORE
    cd - 1> /dev/null
  fi
done
