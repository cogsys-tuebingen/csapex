#!/bin/bash
echo "running test coverage"

THISDIR=$(pwd)
ROOT=$(cd ../../../../../; pwd)
DIR=$THISDIR/traces

# build
cd $ROOT/build
make -j 12

# clean
cd $THISDIR
echo "run on $DIR"
lcov --zerocounters --directory $DIR

rm -fr coverage
rm -fr $DIR
rm *.info

# run tests
cd $THISDIR
mkdir $DIR
find $ROOT/build -iname *.gcno | xargs cp -t $DIR

cd $ROOT/build
make test

cd $THISDIR
find $ROOT/build -iname *.gcda | xargs mv -t $DIR
lcov --capture --directory $DIR --output-file $THISDIR/test.info
lcov --extract test.info "*apex*" -o test_extracted.info

# output
cd $THISDIR
genhtml --output-directory $THISDIR/coverage test_extracted.info
gnome-open coverage/index.html
