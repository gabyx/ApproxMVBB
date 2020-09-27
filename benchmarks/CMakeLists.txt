# Add Benchmark ===========================================================================
include(DownloadProject)
download_project(PROJ                googlebenchmark
                 GIT_REPOSITORY      https://github.com/google/benchmark.git
                 GIT_TAG             master
                 UPDATE_DISCONNECTED 1
)

add_subdirectory(${googlebenchmark_SOURCE_DIR} ${googlebenchmark_BINARY_DIR} EXCLUDE_FROM_ALL)
# ============================================================================================

set(ApproxMVBB_TESTS_OUTPUT_FILES_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(ApproxMVBB_TESTS_INPUT_FILES_DIR "${ApproxMVBB_ROOT_DIR}/tests/files")
set(ApproxMVBB_TESTS_INPUT_FILES_ADD_DIR "${ApproxMVBB_ROOT_DIR}/additional/tests/files")
set(ApproxMVBB_TESTS_VALIDATION_FILES_DIR "${ApproxMVBB_ROOT_DIR}/additional/tests/files/validation")

# WRITE CONFIGURATION FILE
configure_file (
  "${ApproxMVBB_ROOT_DIR}/tests/include/TestConfig.hpp.in.cmake"
  ${CMAKE_CURRENT_BINARY_DIR}/include/TestConfig.hpp
)

set(SOURCE_FILES
    ${ApproxMVBB_ROOT_DIR}/tests/src/CommonFunctions.cpp)

set(INCLUDE_FILES
    ${ApproxMVBB_ROOT_DIR}/tests/include/CommonFunctions.hpp)

set(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    ${CMAKE_CURRENT_BINARY_DIR}/include/
    ${ApproxMVBB_ROOT_DIR}/tests/include/)

include(SetTargetCompileOptions)

set(EXEC_NAME1 ApproxMVBB-BenhmarkMVBB)
add_executable(${EXEC_NAME1}  ${CMAKE_CURRENT_SOURCE_DIR}/src/main_mvbbBenchmarks.cpp ${SOURCE_FILES} ${INCLUDE_FILES})
target_include_directories(${EXEC_NAME1} PRIVATE ${INCLUDE_DIRS})
target_link_libraries(${EXEC_NAME1} ApproxMVBB benchmark)
setTargetCompileOptions(${EXEC_NAME1})