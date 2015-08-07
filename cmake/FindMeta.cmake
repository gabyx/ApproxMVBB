# Find the Meta include directory
# The following variables are set if Meta is found.
#  Meta_FOUND        - True when the Meta include directory is found.
#  Meta_INCLUDE_DIR  - The path to where the meta include files are.
#  Meta_TARGET       - If meta is downloaded from source (if the user has not installed it system-wide, 
#                       this target is used for add_dependency for any project using meta
# If Meta is not found, Meta_FOUND is set to false.

find_package(PkgConfig)

include(FindPackageHandleStandardArgs)

if(NOT EXISTS "${Meta_INCLUDE_DIR}")
    message(STATUS "Meta: find meta")
  find_path(Meta_INCLUDE_DIR
    NAMES meta/meta.hpp 
    DOC "Meta library header files"
    )
endif()

if(EXISTS "${Meta_INCLUDE_DIR}")
  set(Meta_TARGET "" CACHE STRING "Meta target" FORCE)
else()

  message(STATUS "Meta: Setup External Projext")
  include(ExternalProject)
  ExternalProject_Add(meta
    GIT_REPOSITORY https://github.com/ericniebler/meta.git
    TIMEOUT 10
    CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    BUILD_COMMAND "" # disable build step
    INSTALL_COMMAND "" # Disable install step
    )
  
  # Specify include dir
  ExternalProject_Get_Property(meta source_dir)
  set(Meta_INCLUDE_DIR ${source_dir}/include CACHE STRING "Meta include directory" FORCE)
  
  set(Meta_TARGET meta CACHE STRING "Meta target" FORCE)
  
endif()

message(STATUS ${Meta_INCLUDE_DIR})
find_package_handle_standard_args(Meta DEFAULT_MSG Meta_INCLUDE_DIR)
mark_as_advanced(Meta_INCLUDE_DIR)