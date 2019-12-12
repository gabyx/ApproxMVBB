macro(setTargetCompileOptions TARGETNAME)

    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")        
        list(APPEND CXX_WARNINGS "-Wall" 
                                "-Wpedantic" 
                                "-Wno-comment" )

        list(APPEND CXX_FLAGS_DEBUG  "-fsanitize=address" 
                                     "-fno-omit-frame-pointer")

    elseif( CMAKE_CXX_COMPILER_ID MATCHES "Clang")

        list(APPEND CXX_WARNINGS    "-Wall" 
                                    "-Wpedantic" 
                                    "-Wno-comment")

        list(APPEND CXX_FLAGS_DEBUG "-fno-omit-frame-pointer")
        
        if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            list(APPEND CXX_FLAGS_DEBUG "-fsanitize=leak"
                                        "-fsanitize=address"
                                        "-fno-omit-frame-pointer")
        endif()

    elseif( CMAKE_CXX_COMPILER_ID MATCHES "MSVC" )
        message(WARNING "MSVC is partially supported (Visual Studio 2013 Update 5), trying to set compiler flags anyway!")
        list(APPEND CXX_FLAGS "/arch:SSE2")
        list(APPEND CXX_WARNINGS "/Wall")
        # We cannot control the FPU unit in 64bit mode, this flag makes MSVC to not use x87 stuff in 64bit (or 32bit) mode
        # so no extended precision should be used, if that does not work, there is the flag /fp:strict
        # NOTE: SSE is already enabled for x64 compiler
        # as per http://stackoverflow.com/questions/1067630/sse2-option-in-visual-c-x64
    else()
        message(FATAL_ERROR "Could not set compile flags for compiler id: ${CMAKE_CXX_COMPILER_ID}")
    endif()

    set_target_properties(${TARGETNAME} PROPERTIES CXX_STANDARD 14)
    target_compile_features(${TARGETNAME} PUBLIC cxx_std_14)
    
    target_compile_options(${TARGETNAME} PRIVATE ${CXX_FLAGS} ${CXX_WARNINGS} )
    target_compile_options(${TARGETNAME} PRIVATE $<$<CONFIG:Debug>:${CXX_FLAGS_DEBUG}> )

    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR 
       CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # with clang 5.0.1: -fsanitize=address produces weird output in lldb for std::string ...
	set_property(TARGET ${TARGETNAME} PROPERTY LINK_FLAGS_DEBUG "-fsanitize=leak -fsanitize=address")
    endif()

endmacro()
