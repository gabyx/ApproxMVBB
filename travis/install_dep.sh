#!/bin/bash

set -e # exit on errors

cd ${ROOT_PATH}

export APPROXMVBB_CACHE_SIGNATURE_FILE="${APPROXMVBB_CACHE_DIR}/APPROXMVBB_DEPS_CACHE_SUCCESSFUL"

# Build the cache only
if [  ! -f "${APPROXMVBB_CACHE_SIGNATURE_FILE}" ] ; then

  echo "ApproxMVBB Build: Build only dependencies! and CACHE them"

  export BUILD_APPROXMVBB="OFF"

  # Install eigen3 =======================================================
  hg clone https://bitbucket.org/eigen/eigen/ ${ROOT_PATH}/eigen3
  cd ${ROOT_PATH}/eigen3 && hg update 3.3
  mkdir ${ROOT_PATH}/eigen3Build
  cd ${ROOT_PATH}/eigen3Build
  export EIGEN_ROOT_DIR=${INSTALL_PREFIX}/include/eigen3 # no idea why this is needed?
  cmake ../eigen3 -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX} -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}"
  sudo make VERBOSE=1 install

  # Install meta =========================================================
  git clone https://github.com/ericniebler/meta.git ${ROOT_PATH}/meta
  sudo cp -r ${ROOT_PATH}/meta/include/* ${INSTALL_PREFIX}/include/
  #ls -a /usr/local/include/meta

  # Install pugixml  =====================================================
  git clone https://github.com/zeux/pugixml.git ${ROOT_PATH}/pugixml
  perl -pi -e 's/\/\/\s*#define\s*PUGIXML_HAS_LONG_LONG/#define PUGIXML_HAS_LONG_LONG/g' ${ROOT_PATH}/pugixml/src/pugiconfig.hpp
  mkdir ${ROOT_PATH}/pugixmlBuild
  cd ${ROOT_PATH}/pugixmlBuild
  cmake ../pugixml -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX} -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}"
  sudo make VERBOSE=1 install

  echo "successful" | sudo tee -a ${APPROXMVBB_CACHE_SIGNATURE_FILE} > /dev/null
  echo "content in ${APPROXMVBB_CACHE_DIR} :"
  ls -al ${APPROXMVBB_CACHE_DIR}

else

  echo "ApproxMVBB Build: Use cached dependencies..."
  echo "content in ${APPROXMVBB_CACHE_DIR} :"
  ls -al ${APPROXMVBB_CACHE_DIR}/{bin,include,lib,share}

  export BUILD_APPROXMVBB="ON"
fi

set +e # exit on errors off