#!/bin/bash
# this script is SOURCED!!!!

set -e # exit on error

# "DEPENDECIES ========================================================================"

#install a newer cmake since at this time Travis only has version 2.8.7
version="v3.10"
name="cmake-3.10.2-Linux-x86_64.sh"
wget --no-check-certificate http://www.cmake.org/files/$version/$name
chmod a+x $name
sudo ./$name --skip-license --prefix=/usr/local/

export INSTALL_PREFIX="$APPROXMVBB_CACHE_DIR"
export PATH="$INSTALL_PREFIX/bin:/usr/local/bin:$PATH"

cd $ROOT_PATH

if [ -n "${GCC_VERSION}" ]; then 
    # https://stackoverflow.com/questions/37603238/fsanitize-not-using-gold-linker-in-gcc-6-1
    export CPPFLAGS="-fuse-ld=gold"
    export CXXFLAGS="$CPPFLAGS"
    export CXX="g++-${GCC_VERSION}" CC="gcc-${GCC_VERSION}"; 
fi
if [ -n "${CLANG_VERSION}" ]; then 

export CXX="clang++-${CLANG_VERSION}" CC="clang-${CLANG_VERSION}"; 
fi

echo "Path set to ${PATH}"
echo "CXX set to ${CXX}"
echo "CC set to ${CC}"

${CXX} --version
cmake --version
echo "cmake at $(which cmake)"



chmod +x $CHECKOUT_PATH/travis/install_dep.sh
# run the command in this process -> env varibales!
. $CHECKOUT_PATH/travis/install_dep.sh
# "DEPENDECIES COMPLETE ================================================================="

set +e # exit on errors off