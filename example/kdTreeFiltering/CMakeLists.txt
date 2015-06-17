

#The name of the project ====================================================================
PROJECT("KdTreeFilteringExample")



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

ADD_DEPENDENCIES(${PROJECT_NAME} generatePredicateInit)

# copy files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND 
          ${CMAKE_COMMAND}
          -E
          copy ${ApproxMVBB_ROOT_DIR}/tests/files/Bunny.txt ${PROJECT_BINARY_DIR}
)