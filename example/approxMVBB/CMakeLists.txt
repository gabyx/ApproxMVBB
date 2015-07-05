

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
if( NOT ApproxMVBB_DEPENDING_TARGETS STREQUAL "")
    add_dependencies(${PROJECT_NAME} ${ApproxMVBB_DEPENDING_TARGETS})
endif()

