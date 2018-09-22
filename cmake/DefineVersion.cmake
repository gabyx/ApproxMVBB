macro(defineVersion ApproxMVBB_ROOT_DIR)
    # Get the version of the project ApproxMVBB
    # if there is a git directory otherwise take it from 
    # the version file
    if(EXISTS "${ApproxMVBB_ROOT_DIR}/.git")
        execute_process(COMMAND "cmake" "-P" "cmake/GetGitRevisionDescriptionExtern.cmake" 
            WORKING_DIRECTORY "${ApproxMVBB_ROOT_DIR}"
            OUTPUT_VARIABLE ApproxMVBB_VERSION ERROR_VARIABLE Error
        )
        MESSAGE(STATUS "ApproxMVBB Version: ${ApproxMVBB_VERSION} extracted from git tags!")
    else()
        file(STRINGS "${ApproxMVBB_ROOT_DIR}/VERSION" ApproxMVBB_VERSION LIMIT_COUNT 1)
        MESSAGE(STATUS "ApproxMVBB Version: ${ApproxMVBB_VERSION} extracted from version file!")
    endif()


    if(Error)
        message(FATAL_ERROR "Error in getting version of ApproxMVBB ${Error}" FATAL)
    endif()

    string(REGEX REPLACE "^.*v([0-9]+)\\..*" "\\1" ApproxMVBB_VERSION_MAJOR "${ApproxMVBB_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.([0-9]+).*" "\\1" ApproxMVBB_VERSION_MINOR "${ApproxMVBB_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" ApproxMVBB_VERSION_PATCH "${ApproxMVBB_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1" ApproxMVBB_VERSION_SHA1 "${ApproxMVBB_VERSION}")
    set(ApproxMVBB_VERSION_STRING "${ApproxMVBB_VERSION_MAJOR}.${ApproxMVBB_VERSION_MINOR}.${ApproxMVBB_VERSION_PATCH}")
    
endmacro()

