MACRO(ApproxMVBB_WRITE_CONFIG_FILE ApproxMVBB_CONFIG_FILE ApproxMVBB_ROOT_DIR )

    # Get the version of the project ApproxMVBB

    include(cmake/GetGitRevisionDescription.cmake)
    git_describe(ApproxMVBB_VERSION --tags --abbrev=0)
    if(NOT ApproxMVBB_VERSION)
        message(FATAL_ERROR "Version number could not be found!" )
    endif()

 
    MESSAGE(STATUS "Version: " ${ApproxMVBB_VERSION})
    string(REGEX REPLACE "^.*v([0-9]+)\\..*" "\\1" ApproxMVBB_VERSION_MAJOR "${ApproxMVBB_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.([0-9]+).*" "\\1" ApproxMVBB_VERSION_MINOR "${ApproxMVBB_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" ApproxMVBB_VERSION_PATCH "${ApproxMVBB_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1" ApproxMVBB_VERSION_SHA1 "${ApproxMVBB_VERSION}")
    set(ApproxMVBB_VERSION_STRING "${ApproxMVBB_VERSION_MAJOR}.${ApproxMVBB_VERSION_MINOR}.${ApproxMVBB_VERSION_PATCH}")
    MESSAGE(STATUS "ApproxMVBB Version: ${ApproxMVBB_VERSION_STRING} extracted from git tags!")

    configure_file(
      ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Config/Config.hpp.in.cmake
      ${ApproxMVBB_CONFIG_FILE}
    )

ENDMACRO()

