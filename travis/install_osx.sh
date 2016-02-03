#!/bin/bash

set -ev # exit on errors

# "DEPENDECIES ========================================================================"

#install prefix
export INSTALL_PREFIX="/usr/local/"

# install additional stuff
brew update
brew tap homebrew/versions
brew link --overwrite gcc49

ls -l "/usr/local/Cellar/gcc/4.9.1/"

export PATH="/usr/local/Cellar/gcc49/4.9.3;$PATH"

brew install isl --HEAD
brew install --cc=gcc-4.9 --HEAD llvm37 --with-asan --with-clang --with-libcxx --rtti --all-targets

find "/usr/local/Cellar/" -name "clang++-3.7"
export PATH="/usr/local/Cellar/llvm35/HEAD/bin/clang++-3.7;$PATH"
 

if [ "$CXX" = "g++" ]; then export CXX="g++-${GCC_VERSION}" CC="gcc-${GCC_VERSION}"; fi
if [ "$CXX" = "clang++" ] ; then export CXX="clang++-${CLANG_VERSION}" CC="clang-${CLANG_VERSION}"; fi

echo "Path set to ${PATH}"
echo "CXX set to ${CXX}"
echo "CC set to ${CC}"

${CXX} --version

chmod +x ./travis/install_dep.sh
. ./travis/install_dep.sh

exit 0

# "DEPENDECIES COMPLETE ================================================================="
 
