#!/bin/bash

if [[ $1 == "" || $2 == "" ]]; then
    echo "usage: $0 <percentage of training files> <percentage of validation files>"
    echo "for example: $0 0.7 0.2"
    exit
fi

shuffled=( $(ls | grep .pgm | sed -r 's/(.[^;]*;)/ \1 /g' | tr " " "\n" | shuf | tr -d " " ) )

n=${#shuffled[@]}

train=$1
validate=$2

training=$(printf "%.0f" $(echo "scale=0; $train * $n" | bc) )
validation=$(printf "%.0f" $(echo "scale=0; $validate * $n" | bc) )
let testing=$n-$training-$validation

echo "training:     " $training
echo "validation:   " $validation
echo "test:         " $testing

echo "Are these numbers okay? [Y/n]"

read ok

if [[ -d train || -d validate || -d test ]]; then
    echo "directories exist, delete them? [y/N]"
    
    read del
    
    if [[ $del == "Y" || $del == "y" ]]; then
        rm -fr train
        rm -fr validate
        rm -fr test
    else
        echo "aborting"
        exit        
    fi
fi


mkdir train
mkdir validate
mkdir test

if [[ $ok == "" || $ok == "y" || $ok == "Y" ]]; then
    for (( i=0; i < $training; ++i )); do
        let idx=i
        echo -ne "\rtraining file $i"
#        cp --link ${shuffled[$idx]} train
        ln ${shuffled[$idx]} train/${shuffled[$idx]}
    done
    echo
    for (( i=0; i < $validation; ++i )); do
        let idx=i+$training
        echo -ne "\rvalidation file $i"
#        cp --link ${shuffled[$idx]} validate
        ln ${shuffled[$idx]} validate/${shuffled[$idx]}
    done
    echo
    for (( i=0; i < $testing; ++i )); do
        let idx=i+$training+$validation
        echo -ne "\rtest file $i"
#        cp --link ${shuffled[$idx]} test
        ln ${shuffled[$idx]} test/${shuffled[$idx]}
    done
    echo
    echo "all done. have a nice day!"
fi

