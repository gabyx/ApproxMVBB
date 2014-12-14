MACRO(ApproxMVBB_WRITE_CONFIG_FILE ApproxMVBB_CONFIG_FILE ApproxMVBB_ROOT_DIR )

    # Get the version of the project ApproxMVBB

    execute_process(COMMAND "cmake" "-P" "cmake/GetGitRevisionDescriptionExtern.cmake" 
        WORKING_DIRECTORY "${ApproxMVBB_ROOT_DIR}"
        OUTPUT_VARIABLE ApproxMVBB_VERSION ERROR_VARIABLE Error
    )

    if(Error)
        message(FATAL_ERROR "Error in getting version of ApproxMVBB ${Error}" FATAL)
    endif()

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

