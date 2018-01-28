include(IncludeInstallFolderPostfix)
macro(INCLUDE_DIAMETER_SOURCE SRC INC INCLUDE_DIRS
                              ROOT_DIR #input
                              ApproxMVBB_ROOT_DIR #input
                              ApproxMVBB_BINARY_DIR #input
                              )
    message(STATUS "Adding  ApproxMVBB - Diameter ======================")
    set(${SRC}
        ${ROOT_DIR}/src/EstimateDiameter.cpp
        ${ROOT_DIR}/src/alloc.cpp
        ${ROOT_DIR}/src/util.cpp
        ${ROOT_DIR}/src/rand.cpp
    )

    set(${INC}
        ${ROOT_DIR}/include/ApproxMVBB/Diameter/EstimateDiameter.hpp
        ${ROOT_DIR}/include/ApproxMVBB/Diameter/TypeSegment.hpp
    )

    set(${INCLUDE_DIRS}
        $<BUILD_INTERFACE:${ROOT_DIR}/include/>
    )

    message(STATUS "====================================================")

endmacro()

macro(INCLUDE_GEOMETRYPREDICATES_SOURCE SRC
                                        INC
                                        INCLUDE_DIRS
                                        ROOT_DIR #input
                                        ApproxMVBB_ROOT_DIR #input
                                        ApproxMVBB_BINARY_DIR #input
                                        )

    message(STATUS "Adding  ApproxMVBB - GeometryPredicates ============")

    set(${INCLUDE_DIRS}
        $<BUILD_INTERFACE:${ROOT_DIR}/include/>
    )

    include(${ApproxMVBB_ROOT_DIR}/cmake/CheckFloatPrecision.cmake)
    check_float_precision()

    message(STATUS "Floating point control:
            HAVE__FPU_SETCW ${HAVE__FPU_SETCW}
            HAVE_FPSETPREC ${HAVE_FPSETPREC}
            HAVE__CONTROLFP ${HAVE__CONTROLFP}
            HAVE__CONTROLFP_S ${HAVE__CONTROLFP_S}
            HAVE_FPU_INLINE_ASM_X86 ${HAVE_FPU_INLINE_ASM_X86}"
            )

    #write config file for this generator
    set(configFile "${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp")
    configure_file(
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp.in.cmake
        ${configFile}
    )
    set(${SRC}
        ${ROOT_DIR}/src/Predicates.cpp
        ${ROOT_DIR}/src/PredicatesInit.cpp
    )

    set(${INC}
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Predicates.hpp
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Rounding.hpp
        ${configFile}
    )

    message(STATUS "====================================================")

endmacro()