

#The name of the library project ====================================================================
PROJECT("ApproxMVBBLib")



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


ADD_LIBRARY(ApproxMVBB SHARED ${SOURCE_FILES} ${INCLUDE_FILES})
set_target_properties(ApproxMVBB PROPERTIES 
    VERSION ${ApproxMVBB_VERSION_STRING} SOVERSION ${ApproxMVBB_VERSION_STRING}
)
ADD_DEPENDENCIES(ApproxMVBB generatePredicateInit)


set(ApproxMVBB_LIBRARY_NAME_REL "${PROJECT_NAME}")
set(ApproxMVBB_LIBRARY_NAME_DBG "${PROJECT_NAME}${CMAKE_DEBUG_POSTFIX}")
set(ApproxMVBB_INCLUDE_CHARATERISTIC_HEADER "Config/Config.hpp")

# Install stuff (default location is not some where on the system! for safty reasons
if(FIRST_CMAKE_RUN)
    SET(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/../install" CACHE STRING "Install prefix (e.g. /usr/local/)" FORCE)
endif()

# Set install directories (if changed CMAKE_INSTALL_PREFIX adjust values)
SET(ApproxMVBB_LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib CACHE STRING "Install dir library")
SET(ApproxMVBB_INC_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB CACHE STRING "Install dir includes")
MARK_AS_ADVANCED(ApproxMVBB_LIB_INSTALL_DIR ApproxMVBB_INCLUDE_INSTALL_DIR )

if(NOT( CMAKE_INSTALL_PREFIX STREQUAL INSTALL_PREFIX_LAST_VALUE ) )
    SET(INSTALL_PREFIX_LAST_VALUE ${CMAKE_INSTALL_PREFIX} CACHE INTERNAL "Last install prefix value")
    MESSAGE(WARNING "Check install dir! They have been changed!")
    UNSET(ApproxMVBB_LIB_INSTALL_DIR)
    UNSET(ApproxMVBB_INCLUDE_INSTALL_DIR)
    SET(ApproxMVBB_LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib CACHE STRING "Install dir library" FORCE)
    SET(ApproxMVBB_INC_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB CACHE STRING "Install dir includes" FORCE)
    MARK_AS_ADVANCED(FORCE ApproxMVBB_LIB_INSTALL_DIR ApproxMVBB_INC_INSTALL_DIR )
endif()

# INSTALLING FILES
MESSAGE( STATUS "Installing library to " "${ApproxMVBB_LIB_INSTALL_DIR}")
install(TARGETS ApproxMVBB LIBRARY DESTINATION ${ApproxMVBB_LIB_INSTALL_DIR})

MESSAGE( STATUS "Installing include folders:" ${ApproxMVBB_INCLUDE_DIR} " to ${ApproxMVBB_INC_INSTALL_DIR}")
install(DIRECTORY  ${ApproxMVBB_INCLUDE_DIR}  DESTINATION ${ApproxMVBB_INC_INSTALL_DIR}
        FILE_PERMISSIONS      OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE)


MESSAGE( STATUS "Installing external include folders:" ${ApproxMVBB_EXTERNAL_INCLUDE_DIRS} " to ${ApproxMVBB_INC_INSTALL_DIR}/ApproxMVBB")
install(DIRECTORY  ${ApproxMVBB_EXTERNAL_INCLUDE_DIRS}
        DESTINATION "${ApproxMVBB_INC_INSTALL_DIR}/ApproxMVBB"
        FILE_PERMISSIONS      OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE)

MESSAGE( STATUS "Installing conmfiguration dependend files:" ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB " to ${ApproxMVBB_INC_INSTALL_DIR}")
install(DIRECTORY  ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB
        DESTINATION ${ApproxMVBB_INC_INSTALL_DIR}
        FILE_PERMISSIONS      OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

#=========================
# Generate ApproxMVBB-config.cmake file which needs to be installed afterwards
SET(ApproxMVBB_LIBRARY_NAME $<TARGET_FILE_NAME:ApproxMVBB>)
SET(ApproxMVBB_CMAKE_CONFIG_FILE "${PROJECT_BINARY_DIR}/approxmvbb-config.cmake") 
configure_file(
  ${PROJECT_SOURCE_DIR}/approxmvbb-config.cmake.in
  ${ApproxMVBB_CMAKE_CONFIG_FILE} @ONLY
)
#=========================
# Generate ApproxMVBB-config-version.cmake file which needs to be installed afterwards
SET(ApproxMVBB_LIBRARY_NAME $<TARGET_FILE_NAME:ApproxMVBB>)
SET(ApproxMVBB_CMAKE_CONFIG_VERSION_FILE "${PROJECT_BINARY_DIR}/approxmvbb-config-version.cmake") 
configure_file(
  ${PROJECT_SOURCE_DIR}/approxmvbb-config-version.cmake.in
  ${ApproxMVBB_CMAKE_CONFIG_VERSION_FILE} @ONLY
)
#=========================


SET(CMAKE_CONFIG_FOLDER ${ApproxMVBB_LIB_INSTALL_DIR}/cmake/ApproxMVBB-${ApproxMVBB_VERSION_STRING})
 MESSAGE( STATUS "Installing cmake config files to ${CMAKE_CONFIG_FOLDER}")
install(FILES  ${ApproxMVBB_CMAKE_CONFIG_FILE} ${ApproxMVBB_CMAKE_CONFIG_VERSION_FILE}
        DESTINATION ${CMAKE_CONFIG_FOLDER}
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
