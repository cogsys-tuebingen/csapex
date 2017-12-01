#!/bin/bash

if [[ ${XDG_CURRENT_DESKTOP} == "Unity" ]]; then
	cd ~
	DIR=$(pwd)
        mkdir -p $DIR/.local/share/applications 2> /dev/null
        mkdir -p $DIR/.local/share/icons 2> /dev/null
        cd -
        # main
        sed "s|\${ws}|$1|" res/csapex.desktop | \
            sed "s|\${name}|$3|" | \
                sed "s|\${exe}|$1/lib/$2/$3|" | \
                        sed "s|\${img}|$DIR/.local/share/icons/application-x-csapex.png|" > \
                        $DIR/.local/share/applications/csapex.desktop
        chmod +x $DIR/.local/share/applications/csapex.desktop
        cp res/apex_logo.png $DIR/.local/share/icons/application-x-csapex.png

        # server
        sed "s|\${ws}|$1|" res/csapex.desktop | \
            sed "s|\${name}|$4|" | \
                sed "s|\${exe}|$1/lib/$2/$4|" | \
                        sed "s|\${img}|$DIR/.local/share/icons/application-x-csapex-server.png|" > \
                        $DIR/.local/share/applications/csapex-server.desktop
        chmod +x $DIR/.local/share/applications/csapex-server.desktop
        cp res/apex_logo_client.png $DIR/.local/share/icons/application-x-csapex-server.png

        # client
        sed "s|\${ws}|$1|" res/csapex.desktop | \
            sed "s|\${name}|$5|" | \
                sed "s|\${exe}|$1/lib/$2/$5|" | \
                        sed "s|\${img}|$DIR/.local/share/icons/application-x-csapex-client.png|" > \
                        $DIR/.local/share/applications/csapex-client.desktop
        chmod +x $DIR/.local/share/applications/csapex-client.desktop
        cp res/apex_logo_server.png $DIR/.local/share/icons/application-x-csapex-client.png

        # mime
	mkdir -p $DIR/.local/share/mime/packages
	touch $DIR/.local/share/mime/packages/application-x-csapex.xml

	echo '<?xml version="1.0" encoding="UTF-8"?>
<mime-info xmlns="http://www.freedesktop.org/standards/shared-mime-info">
    <mime-type type="application/x-csapex">
        <comment>CS::APEX configuration</comment>
        <icon name="application-x-csapex"/>
        <glob-deleteall/>
        <glob pattern="*.apex"/>
    </mime-type>
</mime-info>' > $DIR/.local/share/mime/packages/application-x-csapex.xml

	update-desktop-database ~/.local/share/applications
	update-mime-database    ~/.local/share/mime
fi

mkdir -p ~/.csapex/cfg
cp -r cfg/* ~/.csapex/cfg/
