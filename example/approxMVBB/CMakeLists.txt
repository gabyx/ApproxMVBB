

#The name of the project ====================================================================
PROJECT("ApproxMVBBExample")



SET(SOURCE_FILES
    src/main.cpp
)

SET(INCLUDE_FILES
    ${ApproxMVBB_INC}
)



#include directories
set(INCLUDE_DIRS
    ${EIGEN_INCLUDE_DIR}
    ${ApproxMVBB_INC_DIRS}
)

include_directories(${INCLUDE_DIRS})


ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
target_link_libraries(${PROJECT_NAME} "ApproxMVBB")
if( NOT "${ApproxMVBB_DEPENDING_TARGETS}" STREQUAL "")
    ADD_DEPENDENCIES(${PROJECT_NAME} ${ApproxMVBB_DEPENDING_TARGETS})
endif()

# For some reason, the examples don't under Visual Studio without this:
IF(${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   /DApproxMVBB_BUILD_LIBRARY")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DApproxMVBB_BUILD_LIBRARY")
ENDIF()

