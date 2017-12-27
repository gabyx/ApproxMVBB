#!/bin/bash
# this script is SOURCED!!!!

set -e # exit on errors

# "DEPENDECIES ========================================================================"
cd $ROOT_PATH

#install prefix and path
export INSTALL_PREFIX="$APPROXMVBB_CACHE_DIR"
export PATH="$PATH:$INSTALL_PREFIX/bin"

# travis bug: https://github.com/travis-ci/travis-ci/issues/6307
# rvm get head || true

brew update || echo "suppress failures in order to ignore warnings"
brew tap homebrew/versions || echo "suppress failures in order to ignore warnings"

# eigen3 needs gfortran
brew install gcc || echo "suppress failures in order to ignore warnings"

# Cmake
brew install cmake || echo "suppress failures in order to ignore warnings"
brew upgrade cmake

echo "Path set to ${PATH}"
cmake --version
echo "CXX set to ${CXX}"
echo "CC set to ${CC}"
${CXX} --version

chmod +x $CHECKOUT_PATH/travis/install_dep.sh
. $CHECKOUT_PATH/travis/install_dep.sh

# "DEPENDECIES COMPLETE ================================================================="

# Workaround for https://github.com/travis-ci/travis-ci/issues/6522
set +e # exit on errors off
