#!/bin/bash
# "BUILD ========================================================================"

# "Go to $ROOT_PATH"
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
cmake $CHECKOUT_PATH -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DMYPROJECT_DONTSET_COMPILER_FLAGS_INTERNAL=ON -DCMAKE_CXX_FLAGS="${CXX_FLAGS}" -DCMAKE_EXE_LINKER_FLAGS="${CXX_LINKER_FLAGS}" -DApproxMVBB_FORCE_MSGLOG_LEVEL=2
make VERBOSE=1
make install
cd $ROOT_PATH

# make install and library usage!
echo "Install and test if ApproxMVBB links:"
mkdir $ROOT_PATH/buildLibUsage
cd $ROOT_PATH/buildLibUsage
INSTALL=$(find $ROOT_PATH/build/install/lib/cmake/* -type d)
echo "Install dir= $INSTALL"
cmake $CHECKOUT_PATH/example/libraryUsage -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_CXX_FLAGS="${CXX_FLAGS}" -DCMAKE_EXE_LINKER_FLAGS="${CXX_LINKER_FLAGS}" -DApproxMVBB_DIR=$INSTALL
make VERBOSE=1
cd $ROOT_PATH

#"Run Unit Tests:"
cd $ROOT_PATH/build
make build_and_test

# "BUILD COMPLETE ================================================================"