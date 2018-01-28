add_subdirectory(approxMVBB)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # On Visual Studio, KDTree examples don't compile yet. So only add the required minimal dependencies:
else()
  add_subdirectory(kdTreeFiltering)
endif()
