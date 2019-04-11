#!/bin/bash
# format all .cpp|.hpp files in the repository

export APPROXMVBB_REPO_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

find ${APPROXMVBB_REPO_DIR}/external -type f \( -name "*.hpp" -or  -name "*.cpp" -or -name "*.h" -or  -name "*.c" \) | xargs clang-format -i
find ${APPROXMVBB_REPO_DIR}/include -type f \( -name "*.hpp" -or  -name "*.cpp" \) | xargs clang-format -i
find ${APPROXMVBB_REPO_DIR}/src -type f \( -name "*.hpp" -or  -name "*.cpp" \) | xargs clang-format -i
find ${APPROXMVBB_REPO_DIR}/example -type f \( -name "*.hpp" -or  -name "*.cpp" \)  | xargs clang-format -i
find ${APPROXMVBB_REPO_DIR}/tests -type f \( -name "*.hpp" -or  -name "*.cpp" \)  | xargs clang-format -i
find ${APPROXMVBB_REPO_DIR}/benchmarks -type f \( -name "*.hpp" -or  -name "*.cpp" \)  | xargs clang-format -i
