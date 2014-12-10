

#The name of the project ====================================================================
PROJECT("ApproxMVBBExample")



SET(SOURCE_FILES
    src/main.cpp
)

SET(INCLUDE_FILES
    ${COMMON_INCLUDE}
)



#include directories
set(INCLUDE_DIRS
    ${EIGEN_INCLUDE_DIR}
    ${COMMON_INCLUDE_DIR}
)

include_directories(${INCLUDE_DIRS})


ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
target_link_libraries(${PROJECT_NAME} "ApproxMVBB")


