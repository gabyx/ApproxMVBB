

#The name of the library project ====================================================================
PROJECT("ApproxMVBB")



SET(SOURCE_FILES
    ${COMMON_SOURCE}
)

SET(INCLUDE_FILES
    ${COMMON_INCLUDE}
)

# Building a library means we only export the symbols which have APPROXMVBB_EXPORT set in the Platform.hpp file
# We define all other symbols to be hidden by default as suggested by https://gcc.gnu.org/wiki/Visibility
IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
SET(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS}   -fvisibility=hidden  -DApproxMVBB_BUILD_LIBRARY")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden  -DApproxMVBB_BUILD_LIBRARY")
ENDIF()


#include directories
set(INCLUDE_DIRS
    ${EIGEN_INCLUDE_DIR}
    ${COMMON_INCLUDE_DIR}
)

include_directories(${INCLUDE_DIRS})


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCE_FILES} )

# Install stuff (default location is not some where on the system! for safty reasons
if(FIRST_CMAKE_RUN)
    SET(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/../install" CACHE STRING "Install prefix (e.g. /usr/local/)" FORCE)
endif()

SET(ApproxMVBB_LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib CACHE STRING "Install dir library")
SET(ApproxMVBB_INC_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include CACHE STRING "Install dir includes")
MARK_AS_ADVANCED(ApproxMVBB_LIB_INSTALL_DIR ApproxMVBB_INCLUDE_INSTALL_DIR )

if(NOT( CMAKE_INSTALL_PREFIX STREQUAL INSTALL_PREFIX_LAST_VALUE ) )
    SET(INSTALL_PREFIX_LAST_VALUE ${CMAKE_INSTALL_PREFIX} CACHE INTERNAL "Last install prefix value")
    MESSAGE(WARNING "Check install dir! They have been changed!")
    UNSET(ApproxMVBB_LIB_INSTALL_DIR)
    UNSET(ApproxMVBB_INCLUDE_INSTALL_DIR)
    SET(ApproxMVBB_LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib CACHE STRING "Install dir library" FORCE)
    SET(ApproxMVBB_INC_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include CACHE STRING "Install dir includes" FORCE)
    MARK_AS_ADVANCED(FORCE ApproxMVBB_LIB_INSTALL_DIR ApproxMVBB_INC_INSTALL_DIR )
endif()

# INSTALLING FILES
MESSAGE( STATUS "Installing library to " "${ApproxMVBB_LIB_INSTALL_DIR}")
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${ApproxMVBB_LIB_INSTALL_DIR})

MESSAGE( STATUS "Installing include folders:" ${ApproxMVBB_INCLUDE_DIR} " to ${ApproxMVBB_INC_INSTALL_DIR}")
install(DIRECTORY  ${ApproxMVBB_INCLUDE_DIR}  DESTINATION ${ApproxMVBB_INC_INSTALL_DIR}
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)


MESSAGE( STATUS "Installing external include folders:" ${ApproxMVBB_EXTERNAL_INCLUDE_DIR} " to ${ApproxMVBB_INC_INSTALL_DIR}/ApproxMVBB")
install(DIRECTORY  ${ApproxMVBB_EXTERNAL_INCLUDE_DIR}
        DESTINATION "${ApproxMVBB_INC_INSTALL_DIR}/ApproxMVBB"
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)



MESSAGE( STATUS "Installing config file:" ${ApproxMVBB_CONFIG_FILE} " to ${ApproxMVBB_INCLUDE_INSTALL_DIR}/ApproxMVBB/Config")
install(FILES  ${ApproxMVBB_CONFIG_FILE}
        DESTINATION "${ApproxMVBB_INC_INSTALL_DIR}/ApproxMVBB/Config"
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

#=========================
# Generate ApproxMVBB-config.cmake file which needs to be installed afterwards
SET(ApproxMVBB_LIBRARY_NAME $<TARGET_FILE_NAME:ApproxMVBB>)
SET(ApproxMVBB_CMAKE_CONFIG_FILE "${PROJECT_BINARY_DIR}/ApproxMVBB-config.cmake") 
configure_file(
  ${PROJECT_SOURCE_DIR}/ApproxMVBB-config.cmake.in
  ${ApproxMVBB_CMAKE_CONFIG_FILE} @ONLY
)
#=========================
# Generate ApproxMVBB-config-version.cmake file which needs to be installed afterwards
SET(ApproxMVBB_LIBRARY_NAME $<TARGET_FILE_NAME:ApproxMVBB>)
SET(ApproxMVBB_CMAKE_CONFIG_FILE "${PROJECT_BINARY_DIR}/ApproxMVBB-config-version.cmake") 
configure_file(
  ${PROJECT_SOURCE_DIR}/ApproxMVBB-config-version.cmake.in
  ${ApproxMVBB_CMAKE_CONFIG_FILE} @ONLY
)
#=========================

#Copy ApproxMVBB-config.cmake script to install folder of library/cmake, but first search an already existing cmake|CMake|cMAke folder
#find_path(CMAKE_CONFIG_FOLDER NAMES "cmake" "CMake" "cMake" HINTS ${ApproxMVBB_LIB_INSTALL_DIR} NO_DEFAULT_PATH)
#if(CMAKE_CONFIG_FOLDER)
    #message(STATUS "Installing config file:" ${ApproxMVBB_CMAKE_CONFIG_FILE} " to existing cmake config folder: ${CMAKE_CONFIG_FOLDER}")
#else()
    MESSAGE( STATUS "Installing config file:" ${ApproxMVBB_CMAKE_CONFIG_FILE} " to ${ApproxMVBB_LIB_INSTALL_DIR}/cmake/")
    SET(CMAKE_CONFIG_FOLDER ${ApproxMVBB_LIB_INSTALL_DIR}/cmake/)
#endif()

install(FILES  ${ApproxMVBB_CMAKE_CONFIG_FILE}
        DESTINATION ${CMAKE_CONFIG_FOLDER}
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

install(FILES  ${ApproxMVBB_CMAKE_CONFIG_VERSION_FILE}
        DESTINATION ${CMAKE_CONFIG_FOLDER}
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)