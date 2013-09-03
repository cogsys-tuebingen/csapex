#!/bin/bash
for f in *.jpg
do
./cv_undistort "$f" "undist$f" "params.yaml" 25
done
