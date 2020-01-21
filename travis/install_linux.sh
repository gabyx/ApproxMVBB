#!/bin/bash
# this script is SOURCED!!!!

set -e # exit on error
source "$CHECKOUT_PATH/travis/general.sh"

# "DEPENDECIES ========================================================================"
cd "${ROOT_PATH}"

if [ -n "$USE_GCC" ]; then 
    # https://stackoverflow.com/questions/37603238/fsanitize-not-using-gold-linker-in-gcc-6-1
    updateCIConfig CPPFLAGS "-fuse-ld=gold"
    updateCIConfig CXXFLAGS "$CPPFLAGS"
    
    if [ -n "$GCC_VERSION" ]; then
        updateCIConfig CXX "g++-$GCC_VERSION" 
        updateCIConfig CC "gcc-$GCC_VERSION" 
    else
        updateCIConfig CXX "g++" 
        updateCIConfig CC "gcc"
    fi
fi
if [ -n "$USE_CLANG" ]; then 
    if [ -n "$CLANG_VERSION" ]; then
        updateCIConfig CXX "clang++-$CLANG_VERSION" 
        updateCIConfig CC "clang-$CLANG_VERSION"
    else
        updateCIConfig CXX "clang++" 
        updateCIConfig CC "clang"
    fi
fi

echo "ApproxMVBB CI: Path set to $PATH"
echo "ApproxMVBB CI: CXX set to $CXX"
echo "ApproxMVBB CI: CC set to $CC"

${CXX} --version

# Install newer cmake
version="v3.16"
name="cmake-3.16.3-Linux-x86_64.sh"
wget --no-check-certificate http://www.cmake.org/files/$version/$name
chmod a+x $name
sudo ./$name --skip-license --prefix=/usr/local/

cmake --version
echo "ApproxMVBB CI: make at $(which cmake)"

chmod +x "${CHECKOUT_PATH}/travis/install_dep.sh"
"${CHECKOUT_PATH}/travis/install_dep.sh"
# "DEPENDECIES COMPLETE ================================================================="