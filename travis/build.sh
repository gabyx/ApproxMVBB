#!/bin/bash
# BUILD ========================================================================
# this script is SOURCED!!!!

set -e
source "$CHECKOUT_PATH/travis/general.sh"

echo "ApproxMVBB CI: On branch: ${BRANCH_NAME}"

sourceCIConfig

# Check if we build the project
if [ -z "${BUILD_APPROXMVBB}" ]; then
  echo "ApproxMVBB CI: BUILD_APPROXMVBB not set !"
  exit 1
elif [ "${BUILD_APPROXMVBB}" == "OFF" ]; then
  echo "ApproxMVBB CI: Do not build ApproxMVBB!"
  exit 0
fi

cd "${ROOT_PATH}"
export CXX_FLAGS="-std=c++11"
export CXX_LINKER_FLAGS=""
if [ -z "${BUILD_TYPE}" ]; then export BUILD_TYPE=Release; fi

# make ApproxMVBB
echo "ApproxMVBB CI: Build ApproxMVBB:"
cd "${CHECKOUT_PATH}"
### init submodules
#git submodule init
#git submodule update
## only for hig perf. tests
##- cd addtional/tests/files; cat Lucy* | tar xz

if [ ! -d "${ROOT_PATH}/build" ]; then mkdir "${ROOT_PATH}/build"; fi
cd "${ROOT_PATH}/build"
cmake "${CHECKOUT_PATH}" -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}" \
                      -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
                      -DCMAKE_VERBOSE_MAKEFILE=ON \
                      -DApproxMVBB_USE_OPENMP=OFF \
                      -DApproxMVBB_FORCE_MSGLOG_LEVEL=2

echo "ApproxMVBB CI: ApproxMVBB: cmake cache:"
cat "./CMakeCache.txt" | grep "ApproxMVBB_"

make VERBOSE=1
make install
cd "${ROOT_PATH}"

if [[ ${BRANCH_NAME} == "unitTests" ]]; then

  echo "ApproxMVBB CI: ApproxMVBB: Run unit tests!"
  cd "${ROOT_PATH}/build"
  make build_and_test

else

  # make install and library usage!
  echo "ApproxMVBB CI: Install and test if ApproxMVBB links:"
  mkdir "${ROOT_PATH}/buildLibUsage"
  cd "${ROOT_PATH}/buildLibUsage"
  INSTALL="${ROOT_PATH}/build/install/share/ApproxMVBB/cmake"
  echo "Install dir= ${INSTALL}"
  cmake "${CHECKOUT_PATH}/example/libraryUsage" -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}" \
                                            -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
                                            -DCMAKE_VERBOSE_MAKEFILE=ON \
                                            -DApproxMVBB_DIR="${INSTALL}"
  make VERBOSE=1
  cd "${ROOT_PATH}"

fi

# BUILD COMPLETE ================================================================
set +e