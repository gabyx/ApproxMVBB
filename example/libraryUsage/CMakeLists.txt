cmake_minimum_required(VERSION 3.2)

SET(PROJECT_NAME libUsage)


# only set this variable if ApproxMVBB is not installed system-wide
# set(ApproxMVBB_DIR "path/to/ApproxMVBB/Build/Directory/install/lib/cmake/ApproxMVBB-1.0.8")


# add ApproxMVBB and all its dependencies
find_package(ApproxMVBB REQUIRED)

# add CXX flags from library
LIST(APPEND CMAKE_CXX_FLAGS ${ApproxMVBB_CXX_FLAGS}) 

# add include directory
include_directories(${ApproxMVBB_INCLUDE_DIR})

# add simple executable
ADD_EXECUTABLE(libUsage ./main.cpp)

# link with ApproxMVBB
MESSAGE(STATUS "Linking with: " ${ApproxMVBB_LIBRARIES})
target_link_libraries(libUsage ${ApproxMVBB_LIBRARIES})

# add dependecies (if any)
add_dependencies(libUsage ${ApproxMVBB_DEPENDING_TARGETS})