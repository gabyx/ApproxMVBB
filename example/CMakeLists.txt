ADD_SUBDIRECTORY(approxMVBB)

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # On Visual Studio, KDTree examples don't compile yet. So only add the required minimal dependencies:
ELSE()
  ADD_SUBDIRECTORY(kdTreeFiltering)
ENDIF()
