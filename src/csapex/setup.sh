#!/bin/bash

if [[ ${XDG_CURRENT_DESKTOP} == "Unity" ]]; then
	cd ~
	DIR=$(pwd)
	cd -
	sed "s|\${exe}|/$1/$2/$3|" res/csapex.desktop | \
		sed "s|\${img}|$DIR/.local/share/icons/apex_logo.png|" > \
		$DIR/.local/share/applications/csapex.desktop
	chmod +x $DIR/.local/share/applications/csapex.desktop
	cp res/apex_logo.png $DIR/.local/share/icons
fi

cp -r cfg ~/.csapex
