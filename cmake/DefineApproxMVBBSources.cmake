macro(INCLUDE_ALL_ApproxMVBB_SOURCE
      SRC
      INC
      INCLUDE_DIRS
      DEPENDING_TARGETS # Input variable names
      ApproxMVBB_ROOT_DIR
      ApproxMVBB_BINARY_DIR)


    # Add all external sources/headers
    include(${ApproxMVBB_ROOT_DIR}/cmake/DefineApproxMVBBExternalSources.cmake)
    include_diameter_source(ApproxMVBB_DIAM_SRC
                            ApproxMVBB_DIAM_INC
                            ApproxMVBB_DIAM_INC_DIRS
                            ${ApproxMVBB_ROOT_DIR}/external/Diameter
                            ${ApproxMVBB_ROOT_DIR}
                            ${ApproxMVBB_BINARY_DIR})

    include_geometrypredicates_source( ApproxMVBB_GEOMPRED_SRC
                                       ApproxMVBB_GEOMPRED_INC
                                       ApproxMVBB_GEOMPRED_INC_DIRS
                                       ${ApproxMVBB_ROOT_DIR}/external/GeometryPredicates
                                       ${ApproxMVBB_ROOT_DIR}
                                       ${ApproxMVBB_BINARY_DIR})



    # WRITE CONFIGURATION FILE
    include(WriteConfigFile)
    set(ApproxMVBB_CONFIG_FILE ${ApproxMVBB_BINARY_DIR}/include/ApproxMVBB/Config/Config.hpp)
    message(STATUS "ApproxMVBB: Write config file ${ApproxMVBB_CONFIG_FILE}, ${ApproxMVBB_OPENMP_NTHREADS}")
    ApproxMVBB_WRITE_CONFIG_FILE( ${ApproxMVBB_CONFIG_FILE} ${ApproxMVBB_ROOT_DIR})
    #=========================

    set(${SRC}
        ${ApproxMVBB_ROOT_DIR}/src/ApproxMVBB/Common/MyMatrixTypeDefs.cpp
        ${ApproxMVBB_ROOT_DIR}/src/ApproxMVBB/RandomGenerators.cpp
        ${ApproxMVBB_ROOT_DIR}/src/ApproxMVBB/ConvexHull2D.cpp
        ${ApproxMVBB_ROOT_DIR}/src/ApproxMVBB/MinAreaRectangle.cpp
        ${ApproxMVBB_ROOT_DIR}/src/ApproxMVBB/ProjectedPointSet.cpp
        ${ApproxMVBB_ROOT_DIR}/src/ApproxMVBB/OOBB.cpp
        ${ApproxMVBB_ROOT_DIR}/src/ApproxMVBB/AABB.cpp

        ${ApproxMVBB_DIAM_SRC}
        ${ApproxMVBB_GEOMPRED_SRC}
    )

    set(${INC}
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/AssertionDebug.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/ContainerTag.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/CPUTimer.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/Exception.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/FloatingPointComparision.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/LogDefines.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/MyContainerTypeDefs.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/MyMatrixTypeDefs.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/Platform.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/SfinaeMacros.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/StaticAssert.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/TypeDefs.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/Common/TypeDefsPoints.hpp

        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/AABB.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/AngleFunctions.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/ComputeApproxMVBB.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/ContainerFunctions.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/ConvexHull2D.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/GreatestCommonDivisor.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/KdTree.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/KdTreeXml.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/MakeCoordinateSystem.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/MinAreaRectangle.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/OOBB.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/PointFunctions.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/ProjectedPointSet.hpp
        ${ApproxMVBB_ROOT_DIR}/include/ApproxMVBB/RandomGenerators.hpp

        ${ApproxMVBB_DIAM_INC}
        ${ApproxMVBB_GEOMPRED_INC}
        ${ApproxMVBB_CONFIG_FILE}
    )

    set(${INCLUDE_DIRS}
        $<BUILD_INTERFACE:${ApproxMVBB_GEOMPRED_INC_DIRS}>
        $<BUILD_INTERFACE:${ApproxMVBB_DIAM_INC_DIRS}>
        $<BUILD_INTERFACE:${ApproxMVBB_ROOT_DIR}/include>
        $<BUILD_INTERFACE:${ApproxMVBB_BINARY_DIR}/include>
    )

    # No depending targets
    set(${DEPENDING_TARGETS} "")

    foreach(file ${${INC}})
        getIncludeInstallFolderPostfix(${file} postfix )
        if("${postfix}" STREQUAL "")
            message(FATAL_ERROR "wrong path ${PATH}")
        endif()
        install( FILES ${file} DESTINATION "include/${postfix}" )
    endforeach()    


endmacro()
