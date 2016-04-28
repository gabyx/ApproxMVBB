

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
    ${ApproxMVBB_ROOT_DIR}/tests/include
    ${ApproxMVBB_INC_DIRS}
)

include_directories(${INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
target_link_libraries(${PROJECT_NAME} "ApproxMVBB" ${PUGIXML_LIBRARIES})

add_definitions("-DApproxMVBB_SUPPORT_KDTREE")
IF(ApproxMVBB_SUPPORT_XML)
  add_definitions("-DApproxMVBB_SUPPORT_XML")
ENDIF()

if( NOT "${ApproxMVBB_DEPENDING_TARGETS}" STREQUAL "")
    ADD_DEPENDENCIES(${PROJECT_NAME} ${ApproxMVBB_DEPENDING_TARGETS})
endif()

# copy files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND
          ${CMAKE_COMMAND}
          -E
          copy ${ApproxMVBB_ROOT_DIR}/tests/files/Bunny.txt  ${PROJECT_BINARY_DIR}
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND
          ${CMAKE_COMMAND}
          -E
          copy_directory ${PROJECT_SOURCE_DIR}/python ${PROJECT_BINARY_DIR}
)
