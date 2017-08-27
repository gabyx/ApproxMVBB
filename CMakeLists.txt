#The CMake Minimum version that is required.
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

cmake_policy(SET CMP0054 NEW)


#Detect Build Type if Debug appears in CMAKE_BINARY_DIR path
STRING(REGEX MATCH ".*(Debug|debug).*" DEBUG_MATCH ${CMAKE_BINARY_DIR} )
IF(DEBUG_MATCH)
    SET(CMAKE_BUILD_TYPE Debug CACHE STRING "The build type for makefile based generators")
    SET(PROJECTNAMEPREFIX Debug)
ELSE()
    SET(CMAKE_BUILD_TYPE Release CACHE STRING "The build type for makefile based generators")
    SET(PROJECTNAMEPREFIX Release)
ENDIF()

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/;${CMAKE_MODULE_PATH}")
MESSAGE(STATUS "Module path is now set to: " ${CMAKE_MODULE_PATH} )

# Load important modules
INCLUDE(FirstCMakeRun)


#The name of the project ===============================================
SET(ApproxMVBBProjectName "ApproxMVBB")
IF(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
        SET(ApproxMVBBProjectName  "${ApproxMVBBProjectName}-${PROJECTNAMEPREFIX}")
ENDIF()
MESSAGE(STATUS "Project name is: " ${ApproxMVBBProjectName})
PROJECT(${ApproxMVBBProjectName})

SET(ApproxMVBB_BINARY_DIR ${PROJECT_BINARY_DIR})
SET(ApproxMVBB_ROOT_DIR   ${PROJECT_SOURCE_DIR})

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

MESSAGE(STATUS "Compiler ID is: " ${CMAKE_CXX_COMPILER_ID})
IF(NOT MYPROJECT_DONTSET_COMPILER_FLAGS_INTERNAL)
	IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")

		SET(MYPROJECT_DONTSET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		MESSAGE(STATUS "Setting Values for GNU")
		SET(CMAKE_C_FLAGS "-fmessage-length=0 -fmax-errors=50" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS "-std=c++11 -fmax-errors=50 -Werror=return-type" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g -fsanitize=address -Wall -Wpedantic -Wno-char-subscripts" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)

	ELSEIF ( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" )

		MESSAGE(STATUS "Setting Values for Clang")
		SET(MYPROJECT_DONTSET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		SET(CMAKE_C_FLAGS                "" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_C_FLAGS_DEBUG          "-g -Weverything" CACHE STRING "Flags for C Compiler for debug builds" FORCE)
		SET(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG -w" CACHE STRING "Flags for C Compiler for release minsize builds" FORCE)
		SET(CMAKE_C_FLAGS_RELEASE        "-O3 -DNDEBUG -w" CACHE STRING "Flags for C Compiler for release builds" FORCE)
		SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for C Compiler for release builds with debug info" FORCE)

		SET(CMAKE_CXX_FLAGS                "-std=c++11 -Werror=return-type" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g -fsanitize=leak -fsanitize=address -Weverything -Wpedantic -Wno-deprecated-register -Wno-documentation -Wno-comment -Wno-float-equal -Wno-deprecated -Wno-c++98-compat -Wno-unused-macros" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
		SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG -w" CACHE STRING "Flags for CXX Compiler for release minsize builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG -w" CACHE STRING "Flags for CXX Compiler for release builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for CXX Compiler for release builds with debug info" FORCE)

        # SET(CMAKE_AR      "/usr/bin/local/llvm-ar" CACHE STRING "archiver" FORCE )
        # SET(CMAKE_LINKER  "/usr/bin/local/llvm-ld" CACHE STRING "linker tool" FORCE )
        # SET(CMAKE_NM      "/usr/bin/local/llvm-nm" CACHE STRING "nm tool" FORCE )
        # SET(CMAKE_OBJDUMP "/usr/bin/local/llvm-objdump" CACHE STRING "objdump tool" FORCE )
        # SET(CMAKE_RANLIB  "/usr/bin/local/llvm-ranlib" CACHE STRING "ranlib tool" FORCE )

	ELSEIF ( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel" )

        MESSAGE(STATUS "Setting Values for Intel")
		SET(MYPROJECT_DONTSET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		SET(CMAKE_C_FLAGS                "" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_C_FLAGS_DEBUG          "-g -Wall" CACHE STRING "Flags for C Compiler for debug builds" FORCE)
		SET(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for C Compiler for release minsize builds" FORCE)
		SET(CMAKE_C_FLAGS_RELEASE        "-O3 -DNDEBUG" CACHE STRING "Flags for C Compiler for release builds" FORCE)
		SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for C Compiler for release builds with debug info" FORCE)

		SET(CMAKE_CXX_FLAGS                "-std=c++11 " CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g -Wall" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
		SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release minsize builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for CXX Compiler for release builds with debug info" FORCE)

  ELSEIF ( ${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC" )
    MESSAGE(WARNING "MSVC is partially supported (Visual Studio 2013 Update 5), trying to set compiler flags anyway!")
    # add some flags
		SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "/arch:SSE2" CACHE STRING "Flags for CXX Compiler" FORCE)
    # We cannot control the FPU unit in 64bit mode, this flag makes MSVC to not use x87 stuff in 64bit (or 32bit) mode
    # so no extended precision should be used, if that does not work, there is the flag /fp:strict
		# NOTE: SSE is already enabled for x64 compiler
		# as per http://stackoverflow.com/questions/1067630/sse2-option-in-visual-c-x64
  ENDIF()
ENDIF()

# Add the ccache to the build system
IF(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
FIND_PROGRAM(CCACHE_FOUND ccache)
  IF(CCACHE_FOUND)
      SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
      SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  ENDIF(CCACHE_FOUND)
ENDIF()

#Add some multithreaded build support ==================================
MARK_AS_ADVANCED(MULTITHREADED_BUILD)
SET(MULTITHREADED_BUILD ON CACHE BOOL "Parallel build with as many threads as possible!")
IF(MULTITHREADED_BUILD)
	IF(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
            FILE(COPY ${ApproxMVBB_ROOT_DIR}/cmake/parallelmake.sh DESTINATION ${PROJECT_BINARY_DIR}
                FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
                NO_SOURCE_PERMISSIONS
            )
            SET(CMAKE_MAKE_PROGRAM "${PROJECT_BINARY_DIR}/parallelmake.sh")
            MESSAGE(STATUS "Set make program to ${PROJECT_BINARY_DIR}/parallelmake.sh")
    ELSEIF(MSVC)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" "/MP")
      MESSAGE(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
    ENDIF()
ENDIF()
# ======================================================================


#Optional Stuff ========================================================

MARK_AS_ADVANCED( CMAKE_DEBUG_POSTFIX )
SET(CMAKE_DEBUG_POSTFIX "-dbg" CACHE STRING "Debug postfix for library/executable")

MARK_AS_ADVANCED( ApproxMVBB_BUILD_LIBRARY )
SET(ApproxMVBB_BUILD_LIBRARY ON CACHE BOOL "Build a shared library")

MARK_AS_ADVANCED( ApproxMVBB_BUILD_TESTS)
SET(ApproxMVBB_BUILD_TESTS ON CACHE BOOL "Build the tests")

MARK_AS_ADVANCED( ApproxMVBB_BUILD_EXAMPLE )
SET(ApproxMVBB_BUILD_EXAMPLE ON CACHE BOOL "Build the example, the library is automatically built if this is true")

MARK_AS_ADVANCED( ApproxMVBB_FORCE_MSGLOG_LEVEL)
SET(ApproxMVBB_FORCE_MSGLOG_LEVEL "0" CACHE STRING "Force the message log level (0-3), 0 = use deubg/release settings in LogDefines.hpp!")

MARK_AS_ADVANCED( ApproxMVBB_USE_OPENMP)
SET(ApproxMVBB_USE_OPENMP ON CACHE BOOL "Try to use OpenMp for parallel speedup")
#=======================================================================

# Dependencies =========================================================
# Define Eigen
FIND_PACKAGE(Eigen3 REQUIRED)
SET(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
LIST(APPEND ApproxMVBB_INC_DIRS_DEP ${EIGEN_INCLUDE_DIR})


# Try to find Meta and PugiXML if possible
FIND_PACKAGE(Meta)

IF(META_FOUND)
  LIST(APPEND ApproxMVBB_INC_DIRS_DEP ${Meta_INCLUDE_DIR})
  LIST(APPEND ApproxMVBB_DEPENDING_TARGETS_DEP ${Meta_TARGET})
  SET(ApproxMVBB_SUPPORT_KDTREE ON)
ENDIF()

FIND_PACKAGE(PugiXML)
IF(PUGIXML_FOUND)
  LIST(APPEND ApproxMVBB_INC_DIRS_DEP ${PUGIXML_INCLUDE_DIR} )
  LIST(APPEND ApproxMVBB_LIBRARIES_DEP ${PUGIXML_LIBRARIES})
  SET(ApproxMVBB_SUPPORT_XML ON)
ENDIF()

IF(ApproxMVBB_USE_OPENMP)

  include(ProcessorCount)
  ProcessorCount(NPROCESSES)

  FIND_PACKAGE(OpenMP)
  IF(OpenMP_FOUND)
    SET(MYPROJECT_CXX_FLAGS "${MYPROJECT_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    SET(ApproxMVBB_OPENMP_SUPPORT ON)

    MARK_AS_ADVANCED( ApproxMVBB_OPENMP_USE_NTHREADS)
    SET(ApproxMVBB_OPENMP_USE_NTHREADS OFF CACHE BOOL "If the number of threads should be fixed, otherwise determined at runtime!")

    IF(ApproxMVBB_OPENMP_USE_NTHREADS)
        SET(ApproxMVBB_OPENMP_NTHREADS ${NPROCESSES} CACHE STRING "The number of threads to use.")
        MARK_AS_ADVANCED( ApproxMVBB_OPENMP_NTHREADS )
    ENDIF()

  ELSE()
    SET(ApproxMVBB_OPENMP_SUPPORT ON)
  ENDIF()
ENDIF()
# ======================================================================

IF(ApproxMVBB_BUILD_EXAMPLE)
  SET(ApproxMVBB_BUILD_LIBRARY On CACHE BOOL "Build a shared library" FORCE)
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MYPROJECT_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MYPROJECT_CXX_FLAGS}")
MESSAGE(STATUS "Added '${MYPROJECT_CXX_FLAGS}' to CMAKE_CXX and CMAKE_C_FLAGS: ${CMAKE_CXX_FLAGS} and ${CMAKE_C_FLAGS}")

# Define all MVBB Source files
INCLUDE(DefineApproxMVBBSources)

SET(ApproxMVBB_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/ApproxMVBB")
SET(ApproxMVBB_EXTERNAL_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/external/Diameter/include/ApproxMVBB/Diameter"
                                    "${PROJECT_SOURCE_DIR}/external/GeometryPredicates/include/ApproxMVBB/GeometryPredicates")

#Include all relevant sources
INCLUDE_ALL_APPROXMVBB_SOURCE(  ApproxMVBB_SRC
                                ApproxMVBB_INC
                                ApproxMVBB_INC_DIRS
                                ApproxMVBB_DEPENDING_TARGETS
                                ${PROJECT_SOURCE_DIR} ${ApproxMVBB_BINARY_DIR} )

LIST(APPEND ApproxMVBB_INC_DIRS ${ApproxMVBB_INC_DIRS_DEP})
MESSAGE(STATUS "Include Dirs: ${ApproxMVBB_INC_DIRS}")
SET(ApproxMVBB_LINK_LIBS ${ApproxMVBB_LIBRARIES_DEP})
MESSAGE(STATUS "Linking Libs: ${ApproxMVBB_LINK_LIBS}")
LIST(APPEND ApproxMVBB_DEPENDING_TARGETS ${ApproxMVBB_DEPENDING_TARGETS_DEP})

IF(ApproxMVBB_BUILD_LIBRARY)
    ADD_SUBDIRECTORY(lib)
ENDIF()

IF(ApproxMVBB_BUILD_TESTS)
    enable_testing()
    add_custom_target(build_and_test ${CMAKE_CTEST_COMMAND} -V)
    ADD_SUBDIRECTORY(tests)
ENDIF()

IF(ApproxMVBB_BUILD_EXAMPLE)
ADD_SUBDIRECTORY(example)
ENDIF()
