MACRO(INCLUDE_DIAMETER_SOURCE SRC INC INCLUDE_DIRS
                              ROOT_DIR #input
                              ApproxMVBB_ROOT_DIR #input
                              ApproxMVBB_BINARY_DIR #input
                              )
    message(STATUS "Adding  ApproxMVBB - Diameter ======================")
    SET(${SRC}
        ${ROOT_DIR}/src/EstimateDiameter.cpp
        ${ROOT_DIR}/src/alloc.cpp
        ${ROOT_DIR}/src/util.cpp
        ${ROOT_DIR}/src/rand.cpp
    )

    SET(${INC}
        ${ROOT_DIR}/include/ApproxMVBB/Diameter/EstimateDiameter.hpp
    )

    SET(${INCLUDE_DIRS}
        ${ROOT_DIR}/include/
    )

    message(STATUS "====================================================")

ENDMACRO()

MACRO(INCLUDE_GEOMETRYPREDICATES_SOURCE SRC
                                        INC
                                        INCLUDE_DIRS
                                        ROOT_DIR #input
                                        ApproxMVBB_ROOT_DIR #input
                                        ApproxMVBB_BINARY_DIR #input
                                        )

    message(STATUS "Adding  ApproxMVBB - GeometryPredicates ============")

    SET(${SRC}
        ${ROOT_DIR}/src/Predicates.cpp
        ${ROOT_DIR}/src/PredicatesInit.cpp
    )

    SET(${INC}
        ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Predicates.hpp
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Rounding.hpp
    )

    SET(${INCLUDE_DIRS}
        ${ROOT_DIR}/include/
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
    configure_file(
      ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp.in.cmake
      ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp
    )

    message(STATUS "====================================================")

ENDMACRO()


#MACRO(INCLUDE_PUGIXML_SOURCE  SRC INC INCLUDE_DIRS
                              #ROOT_DIR #input
                              #ApproxMVBB_ROOT_DIR #input
                              #ApproxMVBB_BINARY_DIR #input
                              #)
    #message(STATUS "Adding  ApproxMVBB - PugiXML ======================")
    #SET(${SRC}
        #${ROOT_DIR}/include/ApproxMVBB/pugixml/pugixml.cpp
    #)

    #SET(${INC}
        #${ROOT_DIR}/include/ApproxMVBB/pugixml/pugixml.hpp
    #)

    #SET(${INCLUDE_DIRS}
        #${ROOT_DIR}/include/
    #)

    #message(STATUS "====================================================")

#ENDMACRO()

#MACRO(INCLUDE_META_SOURCE  SRC INC INCLUDE_DIRS
                              #ROOT_DIR #input
                              #ApproxMVBB_ROOT_DIR #input
                              #ApproxMVBB_BINARY_DIR #input
                              #)
    #message(STATUS "Adding  ApproxMVBB - Meta ======================")
    #SET(${SRC}
    #)

    #SET(${INC}
        #${ROOT_DIR}/include/ApproxMVBB/meta/meta.hpp
    #)

    #SET(${INCLUDE_DIRS}
        #${ROOT_DIR}/include/
    #)

    #message(STATUS "====================================================")

#ENDMACRO()
