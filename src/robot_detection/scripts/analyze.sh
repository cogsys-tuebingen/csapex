#!/bin/bash

. ~/.bashrc

export JAVA_HOME=/usr/lib/jvm/java-6-openjdk-amd64
export SONAR_RUNNER_HOME=/usr/local/sonar-runner
export PATH=$PATH:$SONAR_RUNNER_HOME/bin

roscd robot_detection

#INCLUDE=$(cat build/includes.txt)
SOURCE=$(find src -name '*.cpp' -o -name '*.h')

REPORT_PATH=$(cd reports; pwd)

cppcheck -v --enable=all --xml $INCLUDE $SOURCE 2> $REPORT_PATH/cppcheck.xml
/usr/local/cppncss/bin/cppncss -r -v -x -k -f=$REPORT_PATH/cppncss.xml $SOURCE

sonar-runner
