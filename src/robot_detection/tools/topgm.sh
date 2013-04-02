#!/bin/bash

# this script converts images in pwd to an 8bpp grayscale pgm
# it keeps original images
# usage topgm *

if [[ -d pgm ]]; then
    echo "directoriy 'pgm' exist, overwrite existing files? [y/N]"
    
    read del
    
    if [[ $del == "Y" || $del == "y" ]]; then
        echo "overwriting 'pgm'"
    else
        echo "aborting"
        exit        
    fi
else
    mkdir pgm
fi


for i in *
do
  if [[ -f $i ]]; then
      echo -ne "\rconverting:" $i
      convert $i pgm/$i.pgm
  fi
done
echo
echo "all done. have a nice day!"

