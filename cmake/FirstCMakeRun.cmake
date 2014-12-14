#
# Set a FIRST_CMAKE_RUN flag indicating that this is the first CMake run for
# this build directory.
#
# This allows us to override some default cmake cache values, but only on the
# first run.  On further runs the user is free to change these defaults without
# being overriden.
#
IF(NOT NOT_FIRST_CMAKE_RUN)
	SET(FIRST_CMAKE_RUN ON)
	SET(NOT_FIRST_CMAKE_RUN ON CACHE INTERNAL "Indicate that this is not the first CMake run" FORCE)
ELSE()
	SET(FIRST_CMAKE_RUN OFF)
ENDIF()

