#!/bin/bash
echo "running test coverage"

set -x
set -e

SCRIPTDIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
THISDIR=$( pwd )
SRCDIR=$(cd $THISDIR/../; pwd)

if [[ "$BUILD_DIR" == "" ]]; then
    BUILD_DIR=build_coverage
    BUILD_PATH=$ROOT/$BUILD_DIR
fi



if ! [[ $IS_CI ]]; then
    # build
    ROOT=$(cd $THISDIR/../../../../; pwd)
    BUILD_PATH=$ROOT/$BUILD_DIR

    cd $ROOT
    rm -fr $BUILD_DIR
    if ! [[ -d $BUILD_DIR ]]; then
        mkdir -p $BUILD_DIR
        cd $BUILD_PATH
        if [[ -d ../src ]]; then
            cmake ../src -DCMAKE_BUILD_TYPE:=Debug -DENABLE_COVERAGE:=True
        else
            find .. | grep CMakeLists.txt
            cmake .. -DCMAKE_BUILD_TYPE:=Debug -DENABLE_COVERAGE:=True
        fi
    else
        cd $BUILD_PATH
    fi

    make -j $(nproc)

    # clean
    cd $THISDIR
    echo "run on $DIR"

    rm -fr coverage
    rm -fr $DIR
    rm *.info
else
    ROOT=$(pwd)
    BUILD_PATH=$ROOT/$BUILD_DIR
fi

echo "generating baseline"
cd $THISDIR
lcov -c -i -d $BUILD_PATH -o test_base.info --no-external -b $ROOT > /dev/null 2>&1
lcov -r test_base.info $BUILD_PATH/\* $SCRIPTDIR/\* \
     \*/csapex_qt/\* \
     -o test_base_clean.info  > /dev/null 2>&1

# run tests
mkdir $THISDIR/coverage
cd $BUILD_PATH
env CTEST_OUTPUT_ON_FAILURE=1 make test

#source devel/setup.bash
#cd $THISDIR
#rosrun csapex_util csapex_util_test
#rosrun csapex_core_test csapex_core_test

echo "capturing coverage data"
cd $THISDIR
lcov -c -d $BUILD_PATH -o test.info --no-external -b $ROOT > /dev/null 2>&1
lcov -r test.info $BUILD_PATH/\* $SCRIPTDIR/\* \
     \*/csapex_qt/\* \
     -o test_clean.info > /dev/null 2>&1
echo "calculating coverage"
lcov -a test_base_clean.info -a test_clean.info -o test_extracted.info

# output
if ! [[ $IS_CI ]]; then
    cd $THISDIR
    genhtml --output-directory $THISDIR/coverage \
      --demangle-cpp --num-spaces 2 --sort \
      --title "CS::APEX - Test Coverage" \
      --function-coverage --legend \
      test_extracted.info  > /dev/null

    gnome-open coverage/index.html
fi

set +x
