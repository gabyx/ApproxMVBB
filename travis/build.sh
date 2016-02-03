#!/bin/bash
echo "BUILD ========================================================================"
export ROOT_PATH=`pwd`/../;
export CHECKOUT_PATH=`pwd`;
echo "ROOT_PATH= $ROOT_PATH"
echo "CHECKOUT_PATH= $CHECKOUT_PATH"

echo "Path set to ${PATH}"
echo "CXX set to ${CXX}"
echo "CC set to ${CC}"

echo "Go to $CHECKOUT_PATH"
cd $CHECKOUT_PATH
if [ ! -d build ]; then mkdir build; fi
cd build


export CXX_FLAGS="-std=c++11"
export CXX_LINKER_FLAGS=""
if [ -z "$BUILD_TYPE" ]; then export BUILD_TYPE=Release; fi

# make ApproxMVBB
echo "Build ApproxMVBB:"
cmake $CHECKOUT_PATH -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DMYPROJECT_DONTSET_COMPILER_FLAGS_INTERNAL=ON -DCMAKE_CXX_FLAGS="${CXX_FLAGS}" -DCMAKE_EXE_LINKER_FLAGS="${CXX_LINKER_FLAGS}" -DApproxMVBB_FORCE_MSGLOG_LEVEL=2
make VERBOSE=1
make install
cd ..

# make install and library usage!
echo "Install and test if ApproxMVBB links:"
mkdir buildLibUsage
cd buildLibUsage
INSTALL=$(find $CHECKOUT_PATH/build/install/lib/cmake/ApproxMVBB* -type d)
echo "Install dir= $INSTALL"
cmake $CHECKOUT_PATH/example/libraryUsage -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_CXX_FLAGS="${CXX_FLAGS}" -DCMAKE_EXE_LINKER_FLAGS="${CXX_LINKER_FLAGS}" -DApproxMVBB_DIR=$INSTALL
make VERBOSE=1
cd ..

# run unit tests
echo "Run Unit Tests:"
cd build
make build_and_test

echo "BUILD COMPLETE ================================================================"

exit 0