#!/bin/bash

if [[ $1 == "" || $2 == "" || $3 == "" ]]; then
    echo "usage: $0 <number of crops to produce> <width> <height>"
    echo "for example: $0 1000 30 15"
    exit
fi

shuffled=( $(ls | grep .pgm | sed -r 's/(.[^;]*;)/ \1 /g' | tr " " "\n" | shuf | tr -d " " ) )
w=$2
h=$3

n=${#shuffled[@]}

if [[ -d ${w}x${h} ]]; then
    echo "dir '${w}x${h}' exists"
else
    mkdir ${w}x${h}
fi

echo "generating $1 samples"

for (( i=0; i < $1; i++)) ; do
    echo -ne "\rsample $i of $1"
    idx=`expr $i % $n`
    img=${shuffled[idx]} 
    
    IMG_CHARS=$(identify "${img}" 2> /dev/null)
    IMG_CHARS=$(echo "${IMG_CHARS}" | sed -n 's/\(^.*\)\ \([0-9]*\)x\([0-9]*\)\ \(.*$\)/\2 \3/p')
    
    iw=$(echo "${IMG_CHARS}" | awk '{print $1}')
    ih=$(echo "${IMG_CHARS}" | awk '{print $2}')

    ff=$((1+(RANDOM%99)))
    f=$(echo "scale=2; $ff / 400" | bc)
   
    rw=$(printf "%.0f" $(echo "scale=2; $f * $iw" | bc) )
    rh=$(printf "%.0f" $(echo "scale=2; $f * $ih" | bc) )


    x=$((RANDOM%($iw-$rw)))
    y=$((RANDOM%($ih-$rh)))
 
#    echo $x $y $rw $rh
 
    convert -crop ${rw}x${rh}+${x}+${y} $img ${w}x${h}/crop-$i-$img
    convert -resize ${w}x${h}! ${w}x${h}/crop-$i-$img ${w}x${h}/crop-$i-$img
done    

echo
echo "all done. have a nice day!"
