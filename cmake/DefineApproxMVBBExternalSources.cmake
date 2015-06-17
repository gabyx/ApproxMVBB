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
                                        DEPENDING_TARGETS
                                        ROOT_DIR #input 
                                        ApproxMVBB_ROOT_DIR #input
                                        ApproxMVBB_BINARY_DIR #input
                                        ) 

    message(STATUS "Adding  ApproxMVBB - GeometryPredicates ============")
    
    SET(${SRC}
        ${ROOT_DIR}/src/Predicates.cpp
    )

    SET(${INC}
        ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp
        #${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Predicates.hpp
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Rounding.hpp
    )

    SET(${INC_DIRS}
        ${ROOT_DIR}/include/ 
    )
    
    #Add generator project for PredicatesInit.hpp
    
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
    
    # add the executable that will do the generation
    
    SET(TARGET_NAME  ${PROJECT_NAME}_PredInitGenInternal)
    SET(TARGET_NAME2 ${PROJECT_NAME}_PredInitGen)
    
    INCLUDE_DIRECTORIES(${ROOT_DIR}/include/ ${ApproxMVBB_BINARY_DIR}/include/)
    ADD_EXECUTABLE(${TARGET_NAME} ${ROOT_DIR}/src/PredicatesInit.cpp)

    GET_TARGET_PROPERTY(MY_GENERATOR_EXE ${TARGET_NAME} LOCATION)

    # add the custom command that will generate the files
    message(STATUS "Generate : " ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp)
    ADD_CUSTOM_COMMAND(
        OUTPUT  ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp
        COMMAND ${MY_GENERATOR_EXE} 
        WORKING_DIRECTORY ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/
        DEPENDS ${TARGET_NAME}
    )
    
    
    # Create custom 
    ADD_CUSTOM_TARGET(${TARGET_NAME2} DEPENDS ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp)
    SET_SOURCE_FILES_PROPERTIES(${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp PROPERTIES GENERATED True)
    
    SET(${DEPENDING_TARGETS} ${TARGET_NAME2})
    
    message(STATUS "Make project depend on target ${TARGET_NAME2}!")
    message(STATUS "====================================================")

ENDMACRO()


MACRO(INCLUDE_PUGIXML_SOURCE  SRC INC INCLUDE_DIRS
                              ROOT_DIR #input
                              ApproxMVBB_ROOT_DIR #input
                              ApproxMVBB_BINARY_DIR #input
                              )
    message(STATUS "Adding  ApproxMVBB - PugiXML ======================")
    SET(${SRC}
    )

    SET(${INC}
        ${ROOT_DIR}/include/ApproxMVBB/pugixml/pugixml.hpp
    )

    SET(${INCLUDE_DIRS}
        ${ROOT_DIR}/include/
    )
    
    message(STATUS "====================================================")

ENDMACRO()

MACRO(INCLUDE_META_SOURCE  SRC INC INCLUDE_DIRS
                              ROOT_DIR #input
                              ApproxMVBB_ROOT_DIR #input
                              ApproxMVBB_BINARY_DIR #input
                              )
    message(STATUS "Adding  ApproxMVBB - Meta ======================")
    SET(${SRC}
    )

    SET(${INC}
        ${ROOT_DIR}/include/ApproxMVBB/meta/meta.hpp
    )

    SET(${INCLUDE_DIRS}
        ${ROOT_DIR}/include/
    )
    
    message(STATUS "====================================================")

ENDMACRO()
