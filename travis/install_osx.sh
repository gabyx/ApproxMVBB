#!/bin/bash

set -e # exit on errors

# "DEPENDECIES ========================================================================"
cd $ROOT_PATH

#install prefix and path
export INSTALL_PREFIX="/usr/local/"
export PATH=$INSTALL_PREFIX/bin:$PATH


if [ "$CXX" = "g++" ]; then 
  brew update
  brew tap homebrew/versions
  brew install gcc49
  brew link --overwrite gcc49
  export CXX="g++-${GCC_VERSION}" CC="gcc-${GCC_VERSION}"; 
fi


if [ "$CXX" = "clang++" ] ; then 
  brew install isl --HEAD
  brew install --cc=gcc-4.9 --HEAD llvm37 --with-asan --with-clang --with-libcxx --rtti --all-targets
  export CXX="clang++-${CLANG_VERSION}" CC="clang-${CLANG_VERSION}"; 
fi

# Cmake
brew install cmake

echo "Path set to ${PATH}"
cmake --version
echo "CXX set to ${CXX}"
echo "CC set to ${CC}"
${CXX} --version

chmod +x $CHECKOUT_PATH/travis/install_dep.sh
. $CHECKOUT_PATH/travis/install_dep.sh

# "DEPENDECIES COMPLETE ================================================================="
 
