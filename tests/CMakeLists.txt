

#The name of the project ====================================================================
PROJECT("ApproxMVBBTests")

MARK_AS_ADVANCED( ApproxMVBB_TESTS_HIGH_PERFORMANCE)
SET(ApproxMVBB_TESTS_HIGH_PERFORMANCE OFF CACHE BOOL "Switch on high-performance tests, these require a lot of RAM and AdditionalFiles need to be in place")


# WRITE CONFIGURATION FILE
configure_file (
  "${PROJECT_SOURCE_DIR}/include/TestConfig.hpp.in.cmake"
  ${PROJECT_BINARY_DIR}/include/TestConfig.hpp
)
#=========================


SET(SOURCE_FILES
    ${ApproxMVBB_SRC}
    ${PROJECT_SOURCE_DIR}/src/TestFunctions.cpp
    ${PROJECT_SOURCE_DIR}/src/main.cpp
)

SET(INCLUDE_FILES
    ${ApproxMVBB_INC}
    ${PROJECT_SOURCE_DIR}/include/TestFunctions.hpp
    ${PROJECT_SOURCE_DIR}/include/ComputeApproxMVBBTests.hpp
)


#include directories
set(INCLUDE_DIRS
    ${EIGEN_INCLUDE_DIR}
    ${ApproxMVBB_INC_DIRS}
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_BINARY_DIR}/include/
)

include_directories(${INCLUDE_DIRS})


ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
ADD_DEPENDENCIES(${PROJECT_NAME} ${ApproxMVBB_DEPENDING_TARGETS})


add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND 
          ${CMAKE_COMMAND}
          -E
          copy_directory ${PROJECT_SOURCE_DIR}/files/ ${PROJECT_BINARY_DIR}
)

if(ApproxMVBB_TESTS_HIGH_PERFORMANCE)

SET(highperfTestFiles ${ApproxMVBB_ROOT_DIR}/additional/tests/files/Lucy.txt)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND 
          ${CMAKE_COMMAND}
          -E
          copy ${highperfTestFiles} ${PROJECT_BINARY_DIR}
)
endif()


add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND 
          ${CMAKE_COMMAND}
          -E
          copy_directory ${PROJECT_SOURCE_DIR}/python/ ${PROJECT_BINARY_DIR}
)

