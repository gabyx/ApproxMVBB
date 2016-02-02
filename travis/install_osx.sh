#!/bin/bash

#install prefix
export INSTALL_PREFIX= "/usr/local/"

# install additional stuff
brew update
brew tap homebrew/versions
brew install gcc49

brew install isl --HEAD
brew install --cc=gcc-4.9 --HEAD llvm36 --with-asan --with-clang --with-libcxx --rtti --all-targets

export PATH="/usr/local/Cellar/llvm35/HEAD/bin/clang++-3.6;$PATH"
 

if [ "$CXX" = "g++" ]; then export CXX="g++-${GCC_VERSION}" CC="gcc-${GCC_VERSION}"; fi
if [ "$CXX" = "clang++" ] ; then export CXX="clang++-${CLANG_VERSION}" CC="clang-${CLANG_VERSION}"; fi

echo "Path set to ${PATH}"
echo "CXX set to ${CXX}"
echo "CC set to ${CC}"

${CXX} --version
${CXX} -v

chmod +x ./travis/install_dep.sh
./travis/install_dep.sh

exit $?
  
