project("ApproxMVBBExample-MVBB")

add_executable(${PROJECT_NAME}  src/main.cpp )
target_link_libraries(${PROJECT_NAME} "ApproxMVBB::Core")
