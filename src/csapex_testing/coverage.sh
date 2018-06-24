#!/bin/bash
echo "running test coverage"

set -x

THISDIR=$(pwd)
SRCDIR=$(cd ..; pwd)
ROOT=$(cd ../../../../; pwd)
DIR=$THISDIR/traces

# build
cd $ROOT
if ! [[ -d build_coverage ]]; then
	mkdir -p build_coverage
	cd $ROOT/build_coverage
	cmake ../src -DCMAKE_BUILD_TYPE:=Debug -DENABLE_COVERAGE:=1
else
	cd $ROOT/build_coverage
fi
make -j $(nproc)

# clean
cd $THISDIR
echo "run on $DIR"

rm -fr coverage
rm -fr $DIR
rm *.info

#lcov --zerocounters --directory $DIR
echo "generating baseline"
lcov -c -i -d $ROOT/build_coverage -o test_base.info --no-external -b $ROOT > /dev/null
lcov -r test_base.info $ROOT/build_coverage/\* $THISDIR/\* \
         \*/csapex_qt/\* \
         -o test_base_clean.info

# run tests
cd $THISDIR
mkdir $DIR
#find $ROOT/build_coverage -iname *.gcno | xargs cp -t $DIR

cd $ROOT/build_coverage
#make test
devel/setup.bash
rosrun csapex_util csapex_util_test
rosrun csapex_core_test csapex_core_test

cd $THISDIR
#find $ROOT/build_coverage -iname *.gcda | xargs mv -t $DIR
echo "capturing coverage data"
lcov -c -d $ROOT/build_coverage -o test.info --no-external -b $ROOT > /dev/null
lcov -r test.info $ROOT/build_coverage/\* $THISDIR/\* \
         \*/csapex_qt/\* \
         -o test_clean.info
echo "calculating coverage"
lcov -a test_base_clean.info -a test_clean.info -o test_extracted.info > /dev/null

# output
cd $THISDIR
genhtml --output-directory $THISDIR/coverage \
  --demangle-cpp --num-spaces 2 --sort \
  --title "CS::APEX - Test Coverage" \
  --function-coverage --legend \
  test_extracted.info
gnome-open coverage/index.html

set +x
