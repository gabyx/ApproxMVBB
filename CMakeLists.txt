#The CMake Minimum version that is required.
cmake_minimum_required(VERSION 3.10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/;${CMAKE_MODULE_PATH}")
message(STATUS "Module path is now set to: " ${CMAKE_MODULE_PATH} )

#The name of the project ===============================================
set(ApproxMVBBProjectName "ApproxMVBB")
if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
        set(ApproxMVBBProjectName  "${ApproxMVBBProjectName}-${PROJECTNAMEPREFIX}")
endif()
message(STATUS "Project name is: " ${ApproxMVBBProjectName})
project(${ApproxMVBBProjectName} LANGUAGES CXX C)

set(ApproxMVBB_BINARY_DIR ${PROJECT_BINARY_DIR})
set(ApproxMVBB_ROOT_DIR   ${PROJECT_SOURCE_DIR})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)

# PRECISION ISSUES =====================================================
# From http://christian-seiler.de/projekte/fpmath/
#x86 platforms support two types of floating point instruction sets nowadays:
#The classical x87 floating point unit and the new MMX/SSE instructions that can even perform parallel calculations.
#The characteristics of the x87 FPU are:

    #Different instructions for loading and storing different types of floating point numbers from and to memory.
    #A single instruction set for performing calculations.
    #An internal precision with which all calculations are done.
    #Instructions to change the internal precision
    #Support for single, double and double-extended precision.
#The characteristics of the MMX/SSE instructions are:

    #Different instructions for loading and storing different types of floating point numbers from and to memory.
    #Different instructions for performing caluclations depending on the data types in the registers.
    #Calculation precision is dependent on operhand types.
    #Supports only single and double precision
    #Support for parallel computation.
# What we wont in this library is, only the GeometricPredicates need "double precision" (not extended precision)
# That means either by using only sse instruction set this is enforced or by manually setting the FPU unit to double precision
# so far we manually set it to double precision in the Predicates,
# on Windows (64bit) we cannot change FPU precision, so we use the sse flag to ensure only "double precision"
#=======================================================================

# Add the ccache to the build system
if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
      set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
      set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  endif(CCACHE_FOUND)
endif()

#Add some multithreaded build support ==================================
mark_as_advanced(MULTITHREADED_BUILD)
set(MULTITHREADED_BUILD ON CACHE BOOL "Parallel build with as many threads as possible!")
if(MULTITHREADED_BUILD)
    if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
            file(COPY ${ApproxMVBB_ROOT_DIR}/cmake/parallelmake.sh DESTINATION ${PROJECT_BINARY_DIR}
                FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
                NO_SOURCE_PERMISSIONS
            )
            set(CMAKE_MAKE_PROGRAM "${PROJECT_BINARY_DIR}/parallelmake.sh")
            message(STATUS "Set make program to ${PROJECT_BINARY_DIR}/parallelmake.sh")
    elseif(MSVC)
      add_definitions("/MP")
      message(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
    endif()
endif()
# ======================================================================

#Optional Stuff ========================================================
mark_as_advanced( CMAKE_DEBUG_POSTFIX )
set(CMAKE_DEBUG_POSTFIX "-dbg" CACHE STRING "Debug postfix for library/executable")

mark_as_advanced(ApproxMVBB_BUILD_LIBRARY )
set(ApproxMVBB_BUILD_LIBRARY ON CACHE BOOL "Build a shared library")

mark_as_advanced(ApproxMVBB_BUILD_TESTS)
set(ApproxMVBB_BUILD_TESTS ON CACHE BOOL "Build the tests")

mark_as_advanced(ApproxMVBB_BUILD_BENCHMARKS)
set(ApproxMVBB_BUILD_BENCHMARKS OFF CACHE BOOL "Build the benchmarks (only meaningful in Release!)")

mark_as_advanced(ApproxMVBB_BUILD_EXAMPLE )
set(ApproxMVBB_BUILD_EXAMPLE ON CACHE BOOL "Build the example, the library is automatically built if this is true")

mark_as_advanced(ApproxMVBB_FORCE_MSGLOG_LEVEL)
set(ApproxMVBB_FORCE_MSGLOG_LEVEL "-1" CACHE STRING "Force the message log level (0-3), -1 = use debug/release settings in LogDefines.hpp!")

mark_as_advanced(ApproxMVBB_USE_OPENMP)
set(ApproxMVBB_USE_OPENMP ON CACHE BOOL "Try to use OpenMp for parallel speedup")

mark_as_advanced(ApproxMVBB_BUILD_LIBRARY_STATIC  )
set(ApproxMVBB_BUILD_LIBRARY_STATIC OFF CACHE BOOL "Build a static library")

mark_as_advanced(ApproxMVBB_KDTREE_SUPPORT )
set(ApproxMVBB_KDTREE_SUPPORT ON CACHE BOOL "Build library with KDTree support.")
mark_as_advanced(ApproxMVBB_XML_SUPPORT )
set(ApproxMVBB_XML_SUPPORT OFF CACHE BOOL "Build library with XML support.")

#=======================================================================

# Dependencies =========================================================
find_package(Eigen3 CONFIG REQUIRED)
add_library(eigenLib INTERFACE IMPORTED)
set_property(TARGET eigenLib PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
list(APPEND ApproxMVBB_LIBS_DEP_PUBLIC eigenLib)

if(${ApproxMVBB_KDTREE_SUPPORT})
    find_package(Meta REQUIRED)
    add_library(metaLib INTERFACE IMPORTED)
    set_property(TARGET metaLib PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${Meta_INCLUDE_DIR})
    if( NOT ${Meta_TARGET} STREQUAL "")
        add_dependencies(metaLib ${Meta_TARGET})
    endif()
endif()

if(${ApproxMVBB_XML_SUPPORT})
    find_package(PugiXML REQUIRED)
endif()

if(ApproxMVBB_USE_OPENMP)
    include(ProcessorCount)
    ProcessorCount(NPROCESSES)

    find_package(OpenMP)
    if(OpenMP_FOUND)
        list(APPEND ApproxMVBB_LIBS_DEP_PRIVATE OpenMP::OpenMP_CXX)
        set(ApproxMVBB_OPENMP_SUPPORT ON)

        mark_as_advanced(ApproxMVBB_OPENMP_USE_NTHREADS)
        set(ApproxMVBB_OPENMP_USE_NTHREADS OFF CACHE BOOL "If the number of threads should be fixed, otherwise determined at runtime!")
        if(ApproxMVBB_OPENMP_USE_NTHREADS)
            set(ApproxMVBB_OPENMP_NTHREADS ${NPROCESSES} CACHE STRING "The number of threads to use.")
            mark_as_advanced(ApproxMVBB_OPENMP_NTHREADS )
        endif()
    else()
        set(ApproxMVBB_OPENMP_SUPPORT OFF)
    endif()
endif()
# ======================================================================

if(ApproxMVBB_BUILD_EXAMPLE)
    set(ApproxMVBB_BUILD_LIBRARY On CACHE BOOL "Build a shared library" FORCE)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MYPROJECT_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MYPROJECT_CXX_FLAGS}")
message(STATUS "Added '${MYPROJECT_CXX_FLAGS}' to CMAKE_CXX and CMAKE_C_FLAGS: ${CMAKE_CXX_FLAGS} and ${CMAKE_C_FLAGS}")

# Define Install Directories
# Install stuff (default location is not some where on the system! for safty reasons)
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/install" CACHE STRING "Install prefix (e.g. /usr/local/)" FORCE)
endif()
if(${UNIX})
    include(GNUInstallDirs)
    set(ApproxMVBB_INC_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
    set(ApproxMVBB_RUNTIME_INSTALL_DIR "${CMAKE_INSTALL_BINDIR}")
    set(ApproxMVBB_LIBRARY_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
    set(ApproxMVBB_ARCHIVE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
    set(ApproxMVBB_FRAMEWORK_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")

    set(ApproxMVBB_INSTALL_FULL_LIBDIR "${CMAKE_INSTALL_FULL_LIBDIR}")

    set(ApproxMVBB_CMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/ApproxMVBB/cmake")
    set(ApproxMVBB_ADDITIONAL_FILES_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/ApproxMVBB")

elseif(${WIN32})
    set(ApproxMVBB_INC_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
    set(ApproxMVBB_RUNTIME_INSTALL_DIR "bin")
    set(ApproxMVBB_LIBRARY_INSTALL_DIR "bin")
    set(ApproxMVBB_ARCHIVE_INSTALL_DIR "lib")
    set(ApproxMVBB_FRAMEWORK_INSTALL_DIR "bin")

    set(ApproxMVBB_CMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/cmake")
    set(ApproxMVBB_ADDITIONAL_FILES_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}")
else()
	message(FATAL_ERROR "Could not set install folders for this platform!")
endif()

# Define all MVBB Source files
include(SetTargetCompileOptions)
include(DefineApproxMVBBSources)
set(ApproxMVBB_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/ApproxMVBB")
set(ApproxMVBB_EXTERNAL_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/external/Diameter/include/ApproxMVBB"
                                     "${PROJECT_SOURCE_DIR}/external/GeometryPredicates/include/ApproxMVBB")

#Include all relevant sources
include_all_approxmvbb_source(ApproxMVBB_SRC
                              ApproxMVBB_INC
                              ApproxMVBB_INC_DIRS
                              ApproxMVBB_DEPENDING_TARGETS
                              ${PROJECT_SOURCE_DIR} ${ApproxMVBB_BINARY_DIR} )

set(ApproxMVBB_LICENSE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
set(ApproxMVBB_README_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

message(STATUS "Dependecy Libs (public): ${ApproxMVBB_LIBS_DEP_PUBLIC}")

if(ApproxMVBB_BUILD_LIBRARY)
    add_subdirectory(lib)
endif()

if(ApproxMVBB_BUILD_TESTS)
    enable_testing()
    add_custom_target(build_and_test ${CMAKE_CTEST_COMMAND} "--output-on-failure")
    add_subdirectory(tests)
endif()

if(ApproxMVBB_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

if(ApproxMVBB_BUILD_EXAMPLE)
    add_subdirectory(example)
endif()
