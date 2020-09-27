cmake_minimum_required(VERSION 3.10)

project(libraryUsage)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# This include only set compile options: Use it as a reference!
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}../../cmake")
include(SetTargetCompileOptions)

# only set this variable if ApproxMVBB is not installed system-wide
# set(ApproxMVBB_DIR "path/to/ApproxMVBB/Build/Directory/install/lib/cmake/ApproxMVBB-1.0.8")
get_filename_component( ApproxMVBB_DIR ${ApproxMVBB_DIR} ABSOLUTE )
message(STATUS "ApproxMVBB_DIR:" ${ApproxMVBB_DIR})

# Add ApproxMVBB
find_package(ApproxMVBB REQUIRED COMPONENTS XML_SUPPORT KDTREE_SUPPORT)

# Add simple executable
add_executable(libraryUsage ./main.cpp)
setTargetCompileOptions(libraryUsage)
target_link_libraries(libraryUsage ApproxMVBB::Core)