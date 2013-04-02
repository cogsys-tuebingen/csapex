#!/bin/bash

# this script converts images in pwd to an width x height 8bpp grayscale pgm
# it keeps original images
# usage resizeAll *

if [[ $1 == "" || $2 == "" ]]; then
    echo "usage: $0 <width> <height>"
    echo "for example: $0 30 15"
    exit
fi

export w=$1
export h=$2

dir=${w}x${h}

if [[ -d $dir ]]; then
    echo "directoriy '$dir' exist, overwrite existing files? [y/N]"
    
    read del
    
    if [[ $del == "Y" || $del == "y" ]]; then
        echo "overwriting '$dir'"
    else
        echo "aborting"
        exit        
    fi
else
    mkdir $dir
fi


#echo Uebergabeparameter: $*
# for i
for i in *
do
  if [[ -f $i ]]; then
    echo -ne "\rconverting: "$i
    convert -resize ${w}x${h}! $i ${w}x${h}/$i.pgm
  fi
done
echo
echo "all done. have a nice day!"

