
    
    SET(GEOMPRED_SRC
        ${CMAKE_CURRENT_LIST_DIR}/src/Predicates.cpp
        PARENT_SCOPE
    )

    SET(GEOMPRED_INC
        ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp
        #${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/ApproxMVBB/GeometryPredicates/Predicates.hpp
        ${CMAKE_CURRENT_LIST_DIR}/include/ApproxMVBB/GeometryPredicates/Rounding.hpp
        PARENT_SCOPE
    )
    
    

    SET(GEOMPRED_INC_DIRS
        ${CMAKE_CURRENT_LIST_DIR}/include/ 
        PARENT_SCOPE
    )
    
    #Add generator project for PredicatesInit.hpp
    #PROJECT(PredicatesInitGenerator)
    
    include(CheckFloatPrecision)
    check_float_precision()
        
    #write config file for this generator
    configure_file(
      ${CMAKE_CURRENT_LIST_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp.in.cmake
      ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/Config.hpp 
    )
    
    # add the executable that will do the generation
    message(STATUS ${GEOMPRED_INC_DIRS})
    INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/include/ ${ApproxMVBB_BINARY_DIR}/include/)
    ADD_EXECUTABLE(PredicatesInitGenerator ${CMAKE_CURRENT_LIST_DIR}/src/PredicatesInit.c)

    GET_TARGET_PROPERTY(MY_GENERATOR_EXE PredicatesInitGenerator LOCATION)

    # add the custom command that will generate the files
    message(STATUS "Generate : " ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp)
    ADD_CUSTOM_COMMAND(
        OUTPUT  ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp
        COMMAND ${MY_GENERATOR_EXE} 
        WORKING_DIRECTORY ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/
        DEPENDS PredicatesInitGenerator
    )
    
    
    ADD_CUSTOM_TARGET(generatePredicateInit DEPENDS ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp)
    SET_SOURCE_FILES_PROPERTIES(${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/GeometryPredicates/PredicatesInit.hpp PROPERTIES GENERATED True)