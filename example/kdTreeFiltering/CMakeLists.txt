set(SOURCE_FILES src/main.cpp)

set(INCLUDE_FILES )

set(INCLUDE_DIRS
    ${ApproxMVBB_ROOT_DIR}/tests/include)

include(SetTargetCompileOptions)

add_executable(ApproxMVBBExample-KdTree ${SOURCE_FILES} ${INCLUDE_FILES} )
setTargetCompileOptions(ApproxMVBBExample-KdTree)

# Link with Core, and add KDTreeSupport
target_link_libraries(ApproxMVBBExample-KdTree  ApproxMVBB::Core ApproxMVBB::KdTreeSupport)

if(TARGET ApproxMVBB::XMLSupport)
    target_link_libraries(ApproxMVBBExample-KdTree ApproxMVBB::XMLSupport)
endif()

# copy files
add_custom_command(TARGET ApproxMVBBExample-KdTree POST_BUILD
    COMMAND
          ${CMAKE_COMMAND}
          -E
          copy ${ApproxMVBB_ROOT_DIR}/tests/files/Bunny.txt  ${CMAKE_CURRENT_BINARY_DIR}
)

add_custom_command(TARGET ApproxMVBBExample-KdTree POST_BUILD
    COMMAND
          ${CMAKE_COMMAND}
          -E
          copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/python ${CMAKE_CURRENT_BINARY_DIR}
)
