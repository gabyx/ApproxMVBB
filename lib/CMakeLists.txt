

#The name of the library project ====================================================================
PROJECT("ApproxMVBBLib")

# Options ==============================================================
MARK_AS_ADVANCED( ApproxMVBB_BUILD_LIBRARY_STATIC  )
SET(ApproxMVBB_BUILD_LIBRARY_STATIC OFF CACHE BOOL "Build a static library")

MARK_AS_ADVANCED(ApproxMVBB_LIB_INSTALL_DIR ApproxMVBB_INCLUDE_INSTALL_DIR )
# ======================================================================

SET(SOURCE_FILES
    ${ApproxMVBB_SRC}
)

SET(INCLUDE_FILES
    ${ApproxMVBB_INC}
)

#include directories
SET(INCLUDE_DIRS
    ${EIGEN_INCLUDE_DIR}
    ${ApproxMVBB_INC_DIRS}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

SET(LINK_LIBS ${ApproxMVBB_LINK_LIBS})

# Make targets library shared/static ============================================
IF(ApproxMVBB_BUILD_LIBRARY_STATIC)
ADD_LIBRARY(ApproxMVBB STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
SET_TARGET_PROPERTIES(ApproxMVBB PROPERTIES OUTPUT_NAME ApproxMVBBStatic)
ELSE()
ADD_LIBRARY(ApproxMVBB SHARED ${SOURCE_FILES} ${INCLUDE_FILES})
ENDIF()

IF( NOT "${ApproxMVBB_DEPENDING_TARGETS}" STREQUAL "")
    ADD_DEPENDENCIES(ApproxMVBB ${ApproxMVBB_DEPENDING_TARGETS})
ENDIF()

TARGET_LINK_LIBRARIES(ApproxMVBB ${LINK_LIBS})
SET_TARGET_PROPERTIES(ApproxMVBB PROPERTIES
    VERSION ${ApproxMVBB_VERSION_STRING} SOVERSION ${ApproxMVBB_VERSION_STRING}
)

# set build flag, for import/export specifiers in platform.hpp
# Building a library means we only export the symbols which have APPROXMVBB_EXPORT set in the Platform.hpp file
# We define all other symbols to be hidden by default as suggested by https://gcc.gnu.org/wiki/Visibility
IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    target_compile_options(ApproxMVBB PRIVATE -DApproxMVBB_BUILD_LIBRARY -fvisibility=hidden)
ELSE()
    target_compile_options(ApproxMVBB PRIVATE /DApproxMVBB_BUILD_LIBRARY)
ENDIF()
# ======================================================================

# Make target library for tests ================================================
IF(ApproxMVBB_BUILD_TESTS)
  IF(ApproxMVBB_BUILD_LIBRARY_STATIC)
  ADD_LIBRARY(ApproxMVBBForTestsOnly STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
  SET_TARGET_PROPERTIES(ApproxMVBBForTestsOnly PROPERTIES OUTPUT_NAME ApproxMVBBForTestsOnlyStatic)
  ELSE()
  ADD_LIBRARY(ApproxMVBBForTestsOnly SHARED ${SOURCE_FILES} ${INCLUDE_FILES})
  ENDIF()

  IF( NOT "${ApproxMVBB_DEPENDING_TARGETS}" STREQUAL "")
      ADD_DEPENDENCIES(ApproxMVBBForTestsOnly ${ApproxMVBB_DEPENDING_TARGETS})
  ENDIF()

  TARGET_LINK_LIBRARIES(ApproxMVBBForTestsOnly ${LINK_LIBS})
  SET_TARGET_PROPERTIES(ApproxMVBBForTestsOnly PROPERTIES
      VERSION ${ApproxMVBB_VERSION_STRING} SOVERSION ${ApproxMVBB_VERSION_STRING}
  )

  # set build flag, for import/export specifiers in platform.hpp
  # Building a library means we only export the symbols which have APPROXMVBB_EXPORT set in the Platform.hpp file
  # We define all other symbols to be hidden by default as suggested by https://gcc.gnu.org/wiki/Visibility
  IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
      target_compile_options(ApproxMVBBForTestsOnly PRIVATE -DApproxMVBB_BUILD_LIBRARY -DApproxMVBB_BUILD_TESTS -fvisibility=hidden)
  ELSE()
      target_compile_options(ApproxMVBBForTestsOnly PRIVATE /DApproxMVBB_BUILD_LIBRARY /DApproxMVBB_BUILD_TESTS)
  ENDIF()
ENDIF()
# ======================================================================



SET(ApproxMVBB_LIBRARY_NAME_REL "ApproxMVBB")                       # used for cmake config
SET(ApproxMVBB_LIBRARY_NAME_DBG "ApproxMVBB${CMAKE_DEBUG_POSTFIX}") # used for cmake config
SET(ApproxMVBB_INCLUDE_CHARATERISTIC_HEADER "ApproxMVBB/Config/Config.hpp")

# Install stuff (default location is not some where on the system! for safty reasons
IF(FIRST_CMAKE_RUN)
    SET(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/../install" CACHE STRING "Install prefix (e.g. /usr/local/)" FORCE)
ENDIF()

# Set install directories (if changed CMAKE_INSTALL_PREFIX adjust values)
SET(ApproxMVBB_LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib CACHE STRING "Install dir library")
SET(ApproxMVBB_INC_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB CACHE STRING "Install dir includes")


IF(NOT( CMAKE_INSTALL_PREFIX STREQUAL INSTALL_PREFIX_LAST_VALUE ) )
    SET(INSTALL_PREFIX_LAST_VALUE ${CMAKE_INSTALL_PREFIX} CACHE INTERNAL "Last install prefix value")
    MESSAGE(WARNING "Check install dir! They have been changed!")
    UNSET(ApproxMVBB_LIB_INSTALL_DIR)
    UNSET(ApproxMVBB_INCLUDE_INSTALL_DIR)
    SET(ApproxMVBB_LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib CACHE STRING "Install dir library" FORCE)
    SET(ApproxMVBB_INC_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB CACHE STRING "Install dir includes" FORCE)
    MARK_AS_ADVANCED(FORCE ApproxMVBB_LIB_INSTALL_DIR ApproxMVBB_INC_INSTALL_DIR )
ENDIF()

# INSTALLING FILES ================================================================================================================================
MESSAGE( STATUS "Installing library to " "${ApproxMVBB_LIB_INSTALL_DIR}")
INSTALL(TARGETS ApproxMVBB
        LIBRARY DESTINATION ${ApproxMVBB_LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${ApproxMVBB_LIB_INSTALL_DIR})

MESSAGE( STATUS "Installing include folders:" ${ApproxMVBB_INCLUDE_DIR} " to ${ApproxMVBB_INC_INSTALL_DIR}")

INSTALL(DIRECTORY  ${ApproxMVBB_INCLUDE_DIR}  DESTINATION ${ApproxMVBB_INC_INSTALL_DIR}
        FILE_PERMISSIONS      OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE)


MESSAGE( STATUS "Installing external include folders:" ${ApproxMVBB_EXTERNAL_INCLUDE_DIRS} " to ${ApproxMVBB_INC_INSTALL_DIR}/ApproxMVBB")
INSTALL(DIRECTORY  ${ApproxMVBB_EXTERNAL_INCLUDE_DIRS}
        DESTINATION "${ApproxMVBB_INC_INSTALL_DIR}/ApproxMVBB"
        FILE_PERMISSIONS      OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE)

MESSAGE( STATUS "Installing configuration dependend files:" ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB " to ${ApproxMVBB_INC_INSTALL_DIR}")
INSTALL(DIRECTORY  ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB
        DESTINATION ${ApproxMVBB_INC_INSTALL_DIR}
        FILE_PERMISSIONS      OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ
        DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)



# Make configuration files for cmake ============================================================================

SET(CMAKE_CONFIG_FOLDER ${ApproxMVBB_LIB_INSTALL_DIR}/cmake/ApproxMVBB-${ApproxMVBB_VERSION_STRING})

#=========================
# Generate ApproxMVBB-config.cmake file which needs to be installed afterwards
SET(ApproxMVBB_LIBRARY_NAME $<TARGET_FILE_NAME:ApproxMVBB>)
SET(ApproxMVBB_CMAKE_CONFIG_FILE "${PROJECT_BINARY_DIR}/approxmvbb-config.cmake")
CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/approxmvbb-config.cmake.in
  ${ApproxMVBB_CMAKE_CONFIG_FILE} @ONLY
)
#=========================
# Generate ApproxMVBB-config-version.cmake file which needs to be installed afterwards
SET(ApproxMVBB_LIBRARY_NAME $<TARGET_FILE_NAME:ApproxMVBB>)
SET(ApproxMVBB_CMAKE_CONFIG_VERSION_FILE "${PROJECT_BINARY_DIR}/approxmvbb-config-version.cmake")
CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/approxmvbb-config-version.cmake.in
  ${ApproxMVBB_CMAKE_CONFIG_VERSION_FILE} @ONLY
)
#=========================


MESSAGE( STATUS "Installing cmake config files to ${CMAKE_CONFIG_FOLDER}")
INSTALL(FILES  ${ApproxMVBB_CMAKE_CONFIG_FILE} ${ApproxMVBB_CMAKE_CONFIG_VERSION_FILE}
        DESTINATION ${CMAKE_CONFIG_FOLDER}
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

MESSAGE( STATUS "Installing dependency cmake find scripts for 'eigen3,meta,pugixml' to ${CMAKE_CONFIG_FOLDER}")
INSTALL(FILES  ${ApproxMVBB_ROOT_DIR}/cmake/FindEigen3.cmake
               ${ApproxMVBB_ROOT_DIR}/cmake/FindMeta.cmake
               ${ApproxMVBB_ROOT_DIR}/cmake/FindPugiXML.cmake
        DESTINATION ${CMAKE_CONFIG_FOLDER}
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

# ==================================================================================================================
