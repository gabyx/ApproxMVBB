#!/bin/bash

set -e # exit on errors

# "DEPENDECIES ========================================================================"
cd $ROOT_PATH

#install prefix and path
export INSTALL_PREFIX="/usr/local/"
export PATH=$INSTALL_PREFIX/bin:$PATH

# travis bug: https://github.com/travis-ci/travis-ci/issues/6307
# rvm get head || true

# Cmake
brew install cmake || echo "suppress failures in order to ignore warnings"
# eigen3 needs a fortran compiler
brew install gcc || echo "suppress failures in order to ignore warnings"

echo "Path set to ${PATH}"
cmake --version
echo "CXX set to ${CXX}"
echo "CC set to ${CC}"
${CXX} --version

chmod +x $CHECKOUT_PATH/travis/install_dep.sh
. $CHECKOUT_PATH/travis/install_dep.sh

# "DEPENDECIES COMPLETE ================================================================="
 
# Workaround for https://github.com/travis-ci/travis-ci/issues/6522
set +e
