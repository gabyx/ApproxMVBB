macro(defineVersion ApproxMVBB_ROOT_DIR)
    # Get the version of the project ApproxMVBB
    execute_process(COMMAND "cmake" "-P" "cmake/GetGitRevisionDescriptionExtern.cmake" 
        WORKING_DIRECTORY "${ApproxMVBB_ROOT_DIR}"
        OUTPUT_VARIABLE ApproxMVBB_VERSION ERROR_VARIABLE Error
    )

    if(Error)
        message(FATAL_ERROR "Error in getting version of ApproxMVBB ${Error}" FATAL)
    endif()

    message(STATUS "ApproxMVBB Version: ${ApproxMVBB_VERSION_STRING} extracted from git tags!")
endmacro()

