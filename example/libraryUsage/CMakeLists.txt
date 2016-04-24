cmake_minimum_required(VERSION 2.8)

SET(PROJECT_NAME libraryUsage)


# only set this variable if ApproxMVBB is not installed system-wide
# set(ApproxMVBB_DIR "path/to/ApproxMVBB/Build/Directory/install/lib/cmake/ApproxMVBB-1.0.8")

get_filename_component( ApproxMVBB_DIR ${ApproxMVBB_DIR} ABSOLUTE )
message(STATUS "ApproxMVBB_DIR:" ${ApproxMVBB_DIR})


# add ApproxMVBB and support for kdTree and XML output
SET(ApproxMVBB_SUPPORT_KDTREE ON)
SET(ApproxMVBB_SUPPORT_XML ON)
find_package(ApproxMVBB REQUIRED)

# add CXX flags from library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ApproxMVBB_CXX_FLAGS}")

# add include directory
include_directories(${ApproxMVBB_INCLUDE_DIR})

# add simple executable
ADD_EXECUTABLE(libraryUsage ./main.cpp)

# link with ApproxMVBB
MESSAGE(STATUS "Linking with: " ${ApproxMVBB_LIBRARIES})
target_link_libraries(libraryUsage ${ApproxMVBB_LIBRARIES})
target_compile_definitions( ${ApproxMVBB_COMPILE_DEFINITIONS})

# add dependecies (if any)
if( NOT "${ApproxMVBB_DEPENDING_TARGETS}" STREQUAL "")
    ADD_DEPENDENCIES(libraryUsage ${ApproxMVBB_DEPENDING_TARGETS})
endif()
