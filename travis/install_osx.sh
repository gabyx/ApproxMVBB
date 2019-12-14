#!/bin/bash
# this script is SOURCED!!!!

set -e # exit on errors

# "DEPENDECIES ========================================================================"
cd "$ROOT_PATH"

#install prefix and path
export INSTALL_PREFIX="$APPROXMVBB_CACHE_DIR"
export PATH="$INSTALL_PREFIX/bin:$PATH"

brew update || echo "suppress failures in order to ignore warnings"

# eigen3 needs gfortran
brew install gcc || echo "suppress failures in order to ignore warnings"
brew link --overwrite gcc

if [[ "${APPLE_CLANG}" != "YES" ]]; then
    brew install llvm || echo "suppress failures in order to ignore warnings"
    brew link --overwrite llvm
    export PATH="/usr/local/opt/llvm/bin:$PATH"
fi

# Cmake
brew install cmake || echo "suppress failures in order to ignore warnings"
brew upgrade cmake
brew link --overwrite cmake

echo "ApproxMVBB CI: Path set to ${PATH}"
echo "ApproxMVBB CI: CXX set to ${CXX}"
echo "ApproxMVBB CI: CC set to ${CC}"

${CXX} --version
cmake --version

chmod +x "$CHECKOUT_PATH/travis/install_dep.sh"
source "$CHECKOUT_PATH/travis/install_dep.sh"

# "DEPENDECIES COMPLETE ================================================================="

# Workaround for https://github.com/travis-ci/travis-ci/issues/6522
set +e # exit on errors off
