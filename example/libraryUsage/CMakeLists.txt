cmake_minimum_required(VERSION 2.8)

SET(PROJECT_NAME libraryUsage)


# only set this variable if ApproxMVBB is not installed system-wide
# set(ApproxMVBB_DIR "path/to/ApproxMVBB/Build/Directory/install/lib/cmake/ApproxMVBB-1.0.8")

get_filename_component( ApproxMVBB_DIR ${ApproxMVBB_DIR} ABSOLUTE )
message(STATUS "ApproxMVBB_DIR:" ${ApproxMVBB_DIR})


# add ApproxMVBB and all its dependencies
find_package(ApproxMVBB REQUIRED)

# add CXX flags from library
LIST(APPEND CMAKE_CXX_FLAGS ${ApproxMVBB_CXX_FLAGS}) 

# add include directory
include_directories(${ApproxMVBB_INCLUDE_DIR})

# add simple executable
ADD_EXECUTABLE(libraryUsage ./main.cpp)

# link with ApproxMVBB
MESSAGE(STATUS "Linking with: " ${ApproxMVBB_LIBRARIES})
target_link_libraries(libraryUsage ${ApproxMVBB_LIBRARIES})

# add dependecies (if any)
if( NOT ApproxMVBB_DEPENDING_TARGETS STREQUAL "")
    add_dependencies(libraryUsage ${ApproxMVBB_DEPENDING_TARGETS})
endif()
