macro(setTargetCompileOptions TARGETNAME)

    if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
        set(CXX_FLAGS_PUBLIC "-std=c++14" CACHE STRING "Flags for CXX Compiler" FORCE)
        
        set(CXX_WARNINGS    "-Wall" 
                            "-Wpedantic" 
                            "-Wno-comment" 
                            "-Wno-deprecated" 
                            "-Wno-c++98-compat"
                            "-Wno-c++98-compat-pedantic" 
                            "-Wno-unused-macros")

        set(CXX_FLAGS_DEBUG  "-fsanitize=address" 
                             "-fno-omit-frame-pointer")

    elseif( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR 
            ${CMAKE_CXX_COMPILER_ID} STREQUAL "AppleClang" )

        set(CXX_FLAGS_PUBLIC "-std=c++14")

        set(CXX_WARNINGS    "-Wall" 
                            "-Wpedantic" 
                            "-Wno-comment" 
                            "-Wno-deprecated" 
                            "-Wno-c++98-compat"
                            "-Wno-c++98-compat-pedantic" 
                            "-Wno-unused-macros")

        set(CXX_FLAGS_DEBUG "-fno-omit-frame-pointer")
        
        if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
            list(APPEND CXX_FLAGS_DEBUG "-fsanitize=leak"
                                        "-fsanitize=address"
                                        "-fno-omit-frame-pointer")
        endif()

    elseif( ${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC" )
        message(WARNING "MSVC is partially supported (Visual Studio 2013 Update 5), trying to set compiler flags anyway!")
        set(CXX_PUBLIC_FLAGS "-std=c++14")
        set(CXX_FLAGS "/arch:SSE2" "/Wall")
        # We cannot control the FPU unit in 64bit mode, this flag makes MSVC to not use x87 stuff in 64bit (or 32bit) mode
        # so no extended precision should be used, if that does not work, there is the flag /fp:strict
        # NOTE: SSE is already enabled for x64 compiler
        # as per http://stackoverflow.com/questions/1067630/sse2-option-in-visual-c-x64
    else()
        message(FATAL_ERROR "Could not set compile flags for compiler id: ${CMAKE_CXX_COMPILER_ID}")
    endif()

    target_compile_options(${TARGETNAME} PUBLIC ${CXX_FLAGS_PUBLIC} )
    target_compile_options(${TARGETNAME} PRIVATE ${CXX_FLAGS} ${CXX_WARNINGS} )
    target_compile_options(${TARGETNAME} PRIVATE $<$<CONFIG:Debug>:${CXX_FLAGS_DEBUG}> )
    target_compile_options(${TARGETNAME} PRIVATE $<$<CONFIG:Release>:${CXX_FLAGS_RELEASE}> )
    target_compile_options(${TARGETNAME} PRIVATE $<$<CONFIG:MinSizeRel>:${CXX_FLAGS_MINSIZEREL}> )
    target_compile_options(${TARGETNAME} PRIVATE $<$<CONFIG:RelWithDebInfo>:${CXX_FLAGS_RELWITHDEBINFO}> )

    if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR 
       ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
        # with clang 5.0.1: -fsanitize=address produces weird output in lldb for std::string ...
        set_property(TARGET ${TARGETNAME} PROPERTY LINK_FLAGS "-fsanitize=leak -fsanitize=address")
    endif()

endmacro()