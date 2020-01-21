#!/bin/bash
# this script is SOURCED!!!!

set -e # exit on errors
source "$CHECKOUT_PATH/travis/general.sh"

# "DEPENDECIES ========================================================================"
cd "$ROOT_PATH"

# GNU sed
brew install gnu-sed || echo "suppress failures in order to ignore warnings"
PATH="/usr/local/opt/gnu-sed/libexec/gnubin:$PATH"
updateCIConfig PATH "$PATH"

brew update || echo "suppress failures in order to ignore warnings"

# eigen3 needs gfortran
brew install gcc || echo "suppress failures in order to ignore warnings"
brew link --overwrite gcc

if [[ "${APPLE_CLANG}" != "YES" ]]; then
    brew install llvm || echo "suppress failures in order to ignore warnings"
    brew link --overwrite llvm
    updateCIConfig PATH "/usr/local/opt/llvm/bin:$PATH"
fi

# Cmake
brew install cmake || echo "suppress failures in order to ignore warnings"
brew upgrade cmake
brew link --overwrite cmake

# Eigen
brew install eigen

echo "ApproxMVBB CI: Path set to ${PATH}"
echo "ApproxMVBB CI: CXX set to ${CXX}"
echo "ApproxMVBB CI: CC set to ${CC}"

${CXX} --version
cmake --version

chmod +x "$CHECKOUT_PATH/travis/install_dep.sh"
"$CHECKOUT_PATH/travis/install_dep.sh"

# "DEPENDECIES COMPLETE ================================================================="
