#!/bin/bash -i

#PS1='$ '
#source ~/.bashrc

export JAVA_HOME=/usr/lib/jvm/java-6-openjdk-amd64
#export SONAR_RUNNER_HOME=/usr/local/sonar-runner
#export PATH=$PATH:$SONAR_RUNNER_HOME/bin

roscd csapex

INCLUDE=$(roscd; cd ..; cat build/csapex/src/csapex/includes.txt)
SOURCE=$(find . -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.c')
#SOURCE=src/csapex.cpp
mkdir reports -p
REPORT_PATH=$(cd reports; pwd)

#strace cppcheck -v --std=c++11 --enable=all --xml $INCLUDE $SOURCE 
cppcheck -j 4 --platform=unix64 -UWIN32 --force -v --std=c++11 --enable=all --xml $INCLUDE $SOURCE 2> $REPORT_PATH/cppcheck.xml
#/usr/local/cppncss/bin/cppncss -DO_OBJECT -MQ_DECLARE_METATYPE -r -v -x -k -f=$REPORT_PATH/cppncss.xml $SOURCE

sonar-runner
