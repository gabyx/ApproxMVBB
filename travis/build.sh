#!/bin/bash
# BUILD ========================================================================
# this script is SOURCED!!!!

set -e

echo "On branch: $branchName"

# Check if we build the project
if [ "$BUILD_APPROXMVBB" == "OFF" ]; then
  echo "Do not build ApproxMVBB!"
  return
fi

cd $ROOT_PATH
export CXX_FLAGS="-std=c++11"
export CXX_LINKER_FLAGS=""
if [ -z "$BUILD_TYPE" ]; then export BUILD_TYPE=Release; fi

# make ApproxMVBB
echo "Build ApproxMVBB:"
cd $CHECKOUT_PATH
### init submodules
#git submodule init
#git submodule update
## only for hig perf. tests
##- cd addtional/tests/files; cat Lucy* | tar xz

if [ ! -d $ROOT_PATH/build ]; then mkdir $ROOT_PATH/build; fi
cd $ROOT_PATH/build
cmake $CHECKOUT_PATH  -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" \
                      -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
                      -DCMAKE_VERBOSE_MAKEFILE=ON \
                      -ApproxMVBB_USE_OPENMP=OFF \
                      -DApproxMVBB_FORCE_MSGLOG_LEVEL=2

cat ./CMakeCache.txt | grep "ApproxMVBB_"

make VERBOSE=1
make install
cd $ROOT_PATH

if [[ $branchName == "unitTests" ]]; then

  echo "ApproxMVBB: Run unit tests!"
  cd $ROOT_PATH/build
  make build_and_test

else

  # make install and library usage!
  echo "Install and test if ApproxMVBB links:"
  mkdir $ROOT_PATH/buildLibUsage
  cd $ROOT_PATH/buildLibUsage
  INSTALL="$ROOT_PATH/build/install/share/ApproxMVBB/cmake"
  echo "Install dir= $INSTALL"
  cmake $CHECKOUT_PATH/example/libraryUsage -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" \
                                            -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
                                            -DCMAKE_VERBOSE_MAKEFILE=ON \
                                            -DApproxMVBB_DIR=$INSTALL
  make VERBOSE=1
  cd $ROOT_PATH

fi

# BUILD COMPLETE ================================================================
set +e