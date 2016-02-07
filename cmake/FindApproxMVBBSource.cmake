# - Find ApproxMVBBSource
#
# Users can set the following variables before calling the module:
# ApproxMVBB_SEARCH_PATH - The preferred installation prefix for searching for ApproxMVBB. Set by the user.
#
# ApproxMVBB_INC_DIR - the include dir of ot the ApproxMVBB
# ApproxMVBB_SRC_DIR - the source dir of ot the ApproxMVBB
# ApproxMVBB_CMAKE_DIR - the cmake dir of ot the ApproxMVBB

#Find Include Headers
find_path(ApproxMVBB_INC_DIR 
    NAMES "ApproxMVBB/Config/Config.hpp.in.cmake"
    HINTS ${ApproxMVBB_SEARCH_PATH}
    PATH_SUFFIXES
    "include"
    "ApproxMVBB"
    "ApproxMVBB/include"
    NO_DEFAULT_PATHS
)

#Find Root Dir
get_filename_component(result "${ApproxMVBB_INC_DIR}/../" ABSOLUTE)
set(ApproxMVBB_ROOT_DIR  "${result}" CACHE STRING "ApproxMVBB Root Dir")

#Find Source Dir
set(ApproxMVBB_SRC_DIR  "${ApproxMVBB_ROOT_DIR}/src" CACHE STRING "ApproxMVBB Source Dir")
#Find Cmake Dir
set(ApproxMVBB_CMAKE_DIR  "${ApproxMVBB_ROOT_DIR}/cmake" CACHE STRING "ApproxMVBB Cmake Dir")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args("ApproxMVBBSource" DEFAULT_MSG ApproxMVBB_INC_DIR ApproxMVBB_SRC_DIR ApproxMVBB_CMAKE_DIR)

MARK_AS_ADVANCED( ApproxMVBB_FORCE_MSGLOG_LEVEL)
SET(ApproxMVBB_FORCE_MSGLOG_LEVEL "0" CACHE STRING "Force the message log level (0-3), 0 = use deubg/release settings in LogDefines.hpp!")