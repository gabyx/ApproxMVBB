#The CMake Minimum version that is required.
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)




#Detect Build Type if Debug appears in CMAKE_BINARY_DIR path
STRING(REGEX MATCH ".*(Debug|debug).*" DEBUG_MATCH ${CMAKE_BINARY_DIR} )
if(DEBUG_MATCH)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "The build type for makefile based generators")
    SET(PROJECTNAMEPREFIX Debug)
ELSE()
    set(CMAKE_BUILD_TYPE Release CACHE STRING "The build type for makefile based generators")
    SET(PROJECTNAMEPREFIX Release)
ENDIF()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/;${CMAKE_MODULE_PATH}")
message(STATUS "Module path is now set to: " ${CMAKE_MODULE_PATH} )

# Load important modules
include(FirstCMakeRun)


#The name of the project ====================================================================
SET(ApproxMVBBProjectName "ApproxMVBB")
IF(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
        SET(ApproxMVBBProjectName  "${ApproxMVBBProjectName}-${PROJECTNAMEPREFIX}")
ENDIF()

MESSAGE(STATUS "Project name is: " ${ApproxMVBBProjectName})
PROJECT(${ApproxMVBBProjectName})

MESSAGE(STATUS "Compiler ID is: " ${CMAKE_CXX_COMPILER_ID})
IF(NOT MYPROJECT_SET_COMPILER_FLAGS_INTERNAL)
	IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")

		SET(MYPROJECT_SET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		MESSAGE(STATUS "Setting Values for GNU")
		SET(CMAKE_C_FLAGS "-fmessage-length=0" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS "-std=c++11 -Wno-enum-compare" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g -fsanitize=address" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)

	ELSEIF ( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" )

		MESSAGE(STATUS "Setting Values for Clang")
		SET(MYPROJECT_SET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		SET(CMAKE_C_FLAGS                "-Wall" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_C_FLAGS_DEBUG          "-g" CACHE STRING "Flags for C Compiler for debug builds" FORCE)
		SET(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for C Compiler for release minsize builds" FORCE)
		SET(CMAKE_C_FLAGS_RELEASE        "-O4 -DNDEBUG" CACHE STRING "Flags for C Compiler for release builds" FORCE)
		SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for C Compiler for release builds with debug info" FORCE)

        #SET (CMAKE_CXX_FLAGS             "-std=c++11 -ferror-limit=50 -Weverything -Wno-extra-semi -Wno-documentation -pedantic -Wno-conversion -Wno-comment -Wno-unused-parameter -Wno-deprecated-register -Wno-float-equal -Wno-switch -fdiagnostics-show-template-tree -Wno-c++98 -Wno-c++98-compat-pedantic -Wno-deprecated" CACHE STRING "" FORCE)
		SET(CMAKE_CXX_FLAGS                "-std=c++11 -ferror-limit=50 -w" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g -fsanitize=leak -fsanitize=address" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
		SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release minsize builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for CXX Compiler for release builds with debug info" FORCE)

        SET(CMAKE_AR      "/usr/bin/local/llvm-ar" CACHE STRING "archiver" FORCE )
        SET(CMAKE_LINKER  "/usr/bin/local/llvm-ld" CACHE STRING "linker tool" FORCE )
        SET(CMAKE_NM      "/usr/bin/local/llvm-nm" CACHE STRING "nm tool" FORCE )
        SET(CMAKE_OBJDUMP "/usr/bin/local/llvm-objdump" CACHE STRING "objdump tool" FORCE )
        SET(CMAKE_RANLIB  "/usr/bin/local/llvm-ranlib" CACHE STRING "ranlib tool" FORCE )

	ENDIF()
ENDIF()


#Add some multithreaded build support =====================================================================================================
#MARK_AS_ADVANCED(MULTITHREADED_BUILD)
#SET(MULTITHREADED_BUILD 12 CACHE STRING "How many threads are used to build the project")
#if(MULTITHREADED_BUILD AND NOT MULTITHREADED_BUILD_SET)
	#SET(MULTITHREADED_BUILD_SET TRUE CACHE INTERNAL "internal check if we set the multithread stuff already" FORCE)
	#if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
            #SET(CMAKE_MAKE_PROGRAM "${CMAKE_MAKE_PROGRAM} -j${MULTITHREADED_BUILD}")
            #MESSAGE(STATUS "Added arguments to CMAKE_BUILD_TOOL: ${CMAKE_MAKE_PROGRAM}")
    #elseif(MSVC)
      #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
      #MESSAGE(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
    #endif()
#endif()
# ========================================================================================================================================


#Define Eigen ===================================================================================================================================================
find_package(Eigen3 REQUIRED)
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
# ===============================================================================================================================================================

# Check for additional platform stuff =====================================================================================================
include(CheckIncludeFiles)
# Check header files
check_include_files(fpu_control.h HAVE_FPU_CONTROL_H)
# ===============================================================================================================================================================



MARK_AS_ADVANCED( CMAKE_DEBUG_POSTFIX )
SET(CMAKE_DEBUG_POSTFIX "-dbg" CACHE STRING "Debug postfix for library/executable")


MARK_AS_ADVANCED( ApproxMVBB_BUILD_LIBRARY )
set(ApproxMVBB_BUILD_LIBRARY ON CACHE BOOL "Build a shared library")

MARK_AS_ADVANCED( ApproxMVBB_BUILD_TESTS)
set(ApproxMVBB_BUILD_TESTS ON CACHE BOOL "Build the tests")

MARK_AS_ADVANCED( ApproxMVBB_BUILD_EXAMPLE )
set(ApproxMVBB_BUILD_EXAMPLE ON CACHE BOOL "Build the example, the library is automatically built if this is true")

if(ApproxMVBB_BUILD_EXAMPLE)
   set(ApproxMVBB_BUILD_LIBRARY On CACHE BOOL "Build a shared library" FORCE)
endif()






SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MYPROJECT_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MYPROJECT_CXX_FLAGS}")
MESSAGE(STATUS "Added ${MYPROJECT_CXX_FLAGS} to CMAKE_CXX and CMAKE_C_FLAGS: ${CMAKE_CXX_FLAGS} and ${CMAKE_C_FLAGS}")

# Define all MVBB Source files
INCLUDE(DefineApproxMVBBSources)
SET(ApproxMVBB_BINARY_DIR ${PROJECT_BINARY_DIR})
SET(ApproxMVBB_SOURCE_DIR ${PROJECT_SOURCE_DIR})
SET(ApproxMVBB_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/ApproxMVBB")
SET(ApproxMVBB_EXTERNAL_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/external/Diameter/include/ApproxMVBB/Diameter" 
                                    "${PROJECT_SOURCE_DIR}/external/GeometryPredicates/include/ApproxMVBB/GeometryPredicates")
                                    
#Include all relevant sources
INCLUDE_ALL_ApproxMVBB_SOURCE( COMMON_SOURCE COMMON_INCLUDE COMMON_INCLUDE_DIR "${PROJECT_SOURCE_DIR}")

# Add all external sources/headers
ADD_SUBDIRECTORY(external/GeometryPredicates)
ADD_SUBDIRECTORY(external/Diameter)

SET(COMMON_SOURCE ${COMMON_SOURCE} ${DIAM_SRC}   ${GEOMPRED_SRC})
SET(COMMON_INCLUDE ${COMMON_INCLUDE} ${DIAM_INC} ${GEOMPRED_INC})

SET(COMMON_INCLUDE_DIR 
    ${COMMON_INCLUDE_DIR} 
    ${GEOMPRED_INC_DIRS} 
    ${DIAM_INC_DIRS} 
    ${ApproxMVBB_BINARY_DIR}/include/
)


# WRITE CONFIGURATION FILE
INCLUDE(WriteConfigFile)
# Set include file macros to our own headers!
SET(ApproxMVBB_CONFIG_FILE ${PROJECT_BINARY_DIR}/include/ApproxMVBB/Config/Config.hpp) 
ApproxMVBB_WRITE_CONFIG_FILE( ${ApproxMVBB_CONFIG_FILE} ${PROJECT_SOURCE_DIR})
#=========================







if(ApproxMVBB_BUILD_LIBRARY)
    ADD_SUBDIRECTORY(lib)
endif()

if(ApproxMVBB_BUILD_TESTS)
ADD_SUBDIRECTORY(tests)
endif()

if(ApproxMVBB_BUILD_EXAMPLE)
ADD_SUBDIRECTORY(example)
endif()
