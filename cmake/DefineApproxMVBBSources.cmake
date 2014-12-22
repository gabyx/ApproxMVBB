MACRO(INCLUDE_ALL_ApproxMVBB_SOURCE SRC INC INCLUDE_DIRS ROOT_DIR )


    SET(${SRC}
        ${ROOT_DIR}/src/ApproxMVBB/Common/MyMatrixDefs.cpp
              
        ${ROOT_DIR}/src/ApproxMVBB/ConvexHull2D.cpp           
        ${ROOT_DIR}/src/ApproxMVBB/MinAreaRectangle.cpp  
        ${ROOT_DIR}/src/ApproxMVBB/OOBB.cpp              
        ${ROOT_DIR}/src/ApproxMVBB/AABB.cpp   
        
        ${DIAM_SRC}
        ${GEOM_SRC}
    )

    SET(${INC}
        ${ROOT_DIR}/include/ApproxMVBB/Common/MyMatrixTypeDefs.hpp
        ${ROOT_DIR}/include/ApproxMVBB/Common/TypeDefs.hpp
        ${ROOT_DIR}/include/ApproxMVBB/Common/Exception.hpp
        ${ROOT_DIR}/include/ApproxMVBB/Common/StaticAssert.hpp
        ${ROOT_DIR}/include/ApproxMVBB/Common/AssertionDebug.hpp
        ${ROOT_DIR}/include/ApproxMVBB/Common/Platform.hpp
        
        
        ${ROOT_DIR}/include/ApproxMVBB/AABB.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/AngleFunctions.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/ComputeApproxMVBB.hpp 
        ${ROOT_DIR}/include/ApproxMVBB/ContainerFunctions.hpp         
        ${ROOT_DIR}/include/ApproxMVBB/ConvexHull2D.hpp       
        ${ROOT_DIR}/include/ApproxMVBB/GreatestCommonDivisor.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/MakeCoordinateSystem.hpp     
        ${ROOT_DIR}/include/ApproxMVBB/MinAreaRectangle.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/OOBB.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/PointFunctions.hpp     
        ${ROOT_DIR}/include/ApproxMVBB/ProjectedPointSet.hpp
        ${ROOT_DIR}/include/ApproxMVBB/TypeDefsPoints.hpp

        ${DIAM_INC}
        ${GEOM_INC}
    )

    SET(${INCLUDE_DIRS}
        ${ROOT_DIR}/include
        
        ${DIAM_INC_DIRS}
        ${GEOM_INC_DIRS}
    )
ENDMACRO()


MACRO(INCLUDE_ALL_ApproxMVBB_SOURCE_EXTERN SRC INC INCLUDE_DIRS ROOT_DIR )


    SET(${SRC}
        #${ROOT_DIR}/src/ApproxMVBB/Common/MyMatrixDefs.cpp
              
        ${ROOT_DIR}/src/ApproxMVBB/ConvexHull2D.cpp           
        ${ROOT_DIR}/src/ApproxMVBB/MinAreaRectangle.cpp  
        #${ROOT_DIR}/src/ApproxMVBB/OOBB.cpp              
        #${ROOT_DIR}/src/ApproxMVBB/AABB.cpp   
        
        ${DIAM_SRC}
        ${GEOM_SRC}
    )

    SET(${INC}
        #${ROOT_DIR}/include/ApproxMVBB/Common/MyMatrixTypeDefs.hpp
        #${ROOT_DIR}/include/ApproxMVBB/Common/TypeDefs.hpp
        #${ROOT_DIR}/include/ApproxMVBB/Common/Exception.hpp
        #${ROOT_DIR}/include/ApproxMVBB/Common/StaticAssert.hpp
        #${ROOT_DIR}/include/ApproxMVBB/Common/AssertionDebug.hpp
        #${ROOT_DIR}/include/ApproxMVBB/Common/Platform.hpp
        
        
        #${ROOT_DIR}/include/ApproxMVBB/AABB.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/AngleFunctions.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/ComputeApproxMVBB.hpp 
        ${ROOT_DIR}/include/ApproxMVBB/ContainerFunctions.hpp         
        ${ROOT_DIR}/include/ApproxMVBB/ConvexHull2D.hpp       
        ${ROOT_DIR}/include/ApproxMVBB/GreatestCommonDivisor.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/MakeCoordinateSystem.hpp     
        ${ROOT_DIR}/include/ApproxMVBB/MinAreaRectangle.hpp  
        #${ROOT_DIR}/include/ApproxMVBB/OOBB.hpp  
        ${ROOT_DIR}/include/ApproxMVBB/PointFunctions.hpp     
        ${ROOT_DIR}/include/ApproxMVBB/ProjectedPointSet.hpp
        ${ROOT_DIR}/include/ApproxMVBB/TypeDefsPoints.hpp

        ${DIAM_INC}
        ${GEOM_INC}
    )

    SET(${INCLUDE_DIRS}
        ${ROOT_DIR}/include
        
        ${DIAM_INC_DIRS}
        ${GEOM_INC_DIRS}
    )
ENDMACRO()
