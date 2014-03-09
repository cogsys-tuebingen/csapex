#!/bin/bash

if [[ ${XDG_CURRENT_DESKTOP} == "Unity" ]]; then
	cd ~
	DIR=$(pwd)
        mkdir -p $DIR/.local/share/applications 2> /dev/null
        mkdir -p $DIR/.local/share/icons 2> /dev/null
	cd -
	sed "s|\${exe}|/$1/$2/$3|" res/csapex.desktop | \
		sed "s|\${img}|$DIR/.local/share/icons/apex_logo.png|" > \
		$DIR/.local/share/applications/csapex.desktop
	chmod +x $DIR/.local/share/applications/csapex.desktop
        cp res/apex_logo.png $DIR/.local/share/icons/apex_logo.png
fi

cp -r cfg ~/.csapex
