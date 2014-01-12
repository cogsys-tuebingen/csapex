#!/bin/bash
bld=$(tput bold)       # Bold
blu=$(tput setaf 4)    # Blue
rst=$(tput sgr0)       # Text reset

for f in *; do
  if [[ -d $f ]]; then
    cd $f
    echo
    echo "${bld}${blu}checking $f${rst}"
    git push origin master
    cd -
  fi
done
