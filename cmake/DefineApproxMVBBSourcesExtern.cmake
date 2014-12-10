MACRO(INCLUDE_DIAMETER_SOURCE SRC INC INCLUDE_DIRS ROOT_DIR )

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


ENDMACRO()

MACRO(INCLUDE_GEOMETRYPREDICATES_SOURCE SRC INC INCLUDE_DIRS ROOT_DIR )

    SET(${SRC}
        ${ROOT_DIR}/src/Predicates.cpp
    )

    SET(${INC}
        ${ROOT_DIR}/include/ApproxMVBB/GeometryPredicates/Predicates.hpp
    )

    SET(${INCLUDE_DIRS}
        ${ROOT_DIR}/include/
    )

ENDMACRO()