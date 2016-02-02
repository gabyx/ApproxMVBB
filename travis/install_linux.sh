#!/bin/bash


#install a newer cmake since at this time Travis only has version 2.8.7
echo "yes" | sudo add-apt-repository --yes ppa:kalakris/cmake
sudo apt-get update -qq
sudo apt-get install cmake
cmake --version

export INSTALL_PREFIX="/usr/local/"

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