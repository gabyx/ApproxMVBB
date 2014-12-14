# - Find ApproxMVBBSource
#
# Users can set the following variables before calling the module:
# ApproxMVBB_SOURCE_SEARCH_DIR - The preferred installation prefix for searching for ApproxMVBB. Set by the user.
#
# ApproxMVBB_SOURCE_DIR - the source dir of ot the ApproxMVBB


#Find Include Headers
find_path(ApproxMVBB_SOURCE_DIR 
    NAMES "Config.hpp.in.cmake"
    HINTS $ApproxMVBB_SOURCE_SEARCH_DIR$
    PATH_SUFFIXES
    "include"
    "include/ApproxMVBB"
    "include/ApproxMVBB/Config"
    NO_DEFAULT_PATHS
)

message(STATUS ${ApproxMVBB_SOURCE_DIR})
