#!/bin/bash
if [[ -f .gitmodules ]]; then
	echo "updating submodules"
else
	echo "installing submodules"
fi

if [[ -f .gitmodules ]]; then
	git submodule init
	git submodule update
else
	git submodule add gitlab@gitlab.cs.uni-tuebingen.de:csapex/vision-plugins.git plugins/vision_plugins
	git submodule add gitlab@gitlab.cs.uni-tuebingen.de:csapex/backgroundsubtraction.git plugins/background_subtraction
	git submodule add gitlab@gitlab.cs.uni-tuebingen.de:csapex/robot-detection.git plugins/robot_detection
	git submodule add gitlab@gitlab.cs.uni-tuebingen.de:csapex/cip104.git plugins/cip104
	git submodule add gitlab@gitlab.cs.uni-tuebingen.de:csapex/patsy.git plugins/patsy
fi
