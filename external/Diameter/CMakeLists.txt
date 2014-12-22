
    
    SET(DIAM_SRC
        ${CMAKE_CURRENT_LIST_DIR}/src/EstimateDiameter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/alloc.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/util.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/rand.cpp
        PARENT_SCOPE
    )

    SET(DIAM_INC
        ${CMAKE_CURRENT_LIST_DIR}/include/ApproxMVBB/Diameter/EstimateDiameter.hpp
        PARENT_SCOPE
    )

    SET(DIAM_INC_DIRS
        ${CMAKE_CURRENT_LIST_DIR}/include/
        PARENT_SCOPE
    )