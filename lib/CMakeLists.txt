

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

# Install stuff 

MARK_AS_ADVANCED( CMAKE_INSTALL_PREFIX )
SET(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/install" CACHE STRING "Install prefix (e.g. /usr/local/)")


MESSAGE( STATUS "Installing library to " "${CMAKE_INSTALL_PREFIX}/lib")
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION "lib/")

MESSAGE( STATUS "Installing include folders:" ${ApproxMVBB_INCLUDE_DIR} " to ${CMAKE_INSTALL_PREFIX}/include")
install(DIRECTORY  ${ApproxMVBB_INCLUDE_DIR}  DESTINATION "include/"
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)


MESSAGE( STATUS "Installing external include folders:" ${ApproxMVBB_EXTERNAL_INCLUDE_DIR} " to ${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB")
install(DIRECTORY  ${ApproxMVBB_EXTERNAL_INCLUDE_DIR}
        DESTINATION "include/ApproxMVBB"
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)



MESSAGE( STATUS "Installing config file:" ${ApproxMVBB_CONFIG_FILE} " to ${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB/Config")
install(FILES  ${ApproxMVBB_CONFIG_FILE}
        DESTINATION "include/ApproxMVBB/Config"
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
