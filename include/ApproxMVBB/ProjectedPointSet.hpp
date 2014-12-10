// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
// ========================================================================================
#ifndef ApproxMVBB_ProjectedPointSet_hpp
#define ApproxMVBB_ProjectedPointSet_hpp

#include "ApproxMVBB/Common/AssertionDebug.hpp"

#include "ApproxMVBB/Common/TypeDefs.hpp"
#include "ApproxMVBB/TypeDefsPoints.hpp"

#include "ApproxMVBB/PointFunctions.hpp"
#include "ApproxMVBB/MakeCoordinateSystem.hpp"
#include "ApproxMVBB/OOBB.hpp"
#include "ApproxMVBB/MinAreaRectangle.hpp"

namespace ApproxMVBB{
class APPROXMVBB_EXPORT ProjectedPointSet {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DEFINE_MATRIX_TYPES
    DEFINE_POINTS_CONFIG_TYPES

    template<typename Derived>
    OOBB computeMVBBApprox(const Vector3 & zDir,
                           const MatrixBase<Derived> & points,
                           const PREC epsilon) {

        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,Eigen::Dynamic)

        using namespace PointFunctions;
        m_zDir = zDir;
        project(points);
        //std::cout <<"projected points" <<std::endl;

        // Estimate diameter in 2d projective plane
        std::pair<Vector2,Vector2> pp = estimateDiameter<2,Vector2>(m_p,epsilon);


        Vector2 dirX = pp.first - pp.second;
        if( ( pp.second.array() >=  pp.first.array()).all() ) {
            dirX *= -1;
        }
        // If direction zero, use (1,0)
        if( (dirX.array() == 0.0).all() ) {
            dirX.setZero();
            dirX(0)= 1;
        }
        //std::cout <<"estimated 2d diameter: " << dirX.transpose() << " eps: " << epsilon << std::endl;
        // Built Coordinate Trafo from frame K to frame M
        Matrix22 A2_MK;
        dirX.normalize();
        Vector2 dirY(-dirX(1),dirX(0));  // Positive rotation of 90 degrees
        A2_MK.col(0) = dirX;
        A2_MK.col(1) = dirY;
        A2_MK.transposeInPlace();

        AABB2d aabb;
        Vector2 p;
        auto size = m_p.cols();
        for(unsigned int i=0; i < size; ++i) {
            p.noalias() = A2_MK * m_p.col(i); // Transform all points
            aabb.unite(p);
        }

        //        std::cout << "aabb_min: "<<  aabb.m_minPoint.transpose() << std::endl;
        //        std::cout << "aabb_max: "<< aabb.m_maxPoint.transpose() << std::endl;

        Vector3 M_min;
        M_min.head<2>() = aabb.m_minPoint;
        M_min(2) = m_minZValue;

        Vector3 M_max;
        M_max.head<2>() = aabb.m_maxPoint;
        M_max(2) = m_maxZValue;

        // Make coordinate transformation from M frame (Minimum Rectancle)
        // to K frame (Projection Plane);
        Matrix33 A_KM;
        A_KM.setIdentity();
        A_KM.block<2,2>(0,0) =  A2_MK.transpose();

        //        std::cout << "M_min: "<< M_min.transpose() << std::endl;
        //        std::cout << "M_max: "<< M_max.transpose() << std::endl;

        return OOBB(M_min,M_max, m_A_KI.transpose()*A_KM );
    }

    /** Computes the true MVBB by constructing the convex hull in 2d
    * and then building the minimum area rectangle around it and afterwards
    * pulling the rectangle out in direction m_dirZ again which build the MVBB in direction m_dirZ
    */
    template<typename Derived>
    OOBB computeMVBB(const Vector3 & zDir,
                     const MatrixBase<Derived> & points ) {

        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,Eigen::Dynamic)

        using namespace CoordinateSystem;
        m_zDir = zDir;
        project(points);

        // compute minimum area rectangle first
        MinAreaRectangle mar(m_p);
        mar.compute();
        auto box = mar.getMinRectangle();

        // Box coordinates are in K Frame

        Matrix22 A2_KM;
        A2_KM.col(0) = box.m_u.normalized();
        A2_KM.col(1) = box.m_v.normalized();

        Vector2 M_p = A2_KM.transpose()*box.m_p;

        Vector3 M_min;
        M_min.head<2>() = M_p;
        M_min(2) = m_minZValue;

        Vector3 M_max(box.m_u.norm(), box.m_v.norm(),0.0);
        M_max.head<2>() += M_p;
        M_max(2) = m_maxZValue;

        // Make coordinate transformation from M frame (Minimum Rectancle)
        // to K frame (Projection Plane);
        Matrix33 A_IM;
        // Make A_KM
        A_IM.setIdentity();
        A_IM.block<2,2>(0,0) =  A2_KM;
        // Make A_IM;
        A_IM = m_A_KI.transpose()*A_IM; // A_IM = A_IK * A_KM

        return OOBB(M_min,M_max, A_IM );
    }


private:

    template<typename Derived>
    void project(const MatrixBase<Derived> & points) {
        using namespace CoordinateSystem;

        if(points.cols()==0) {
            ERRORMSG("Point set empty!");
        }

        // Generate Orthonormal Bases
        Vector3 xDir,yDir;
        makeCoordinateSystem(m_zDir,xDir,yDir);

        //Make coodinate transform from frame I to K!
        m_A_KI.col(0) = xDir;
        m_A_KI.col(1) = yDir;
        m_A_KI.col(2) = m_zDir;
        m_A_KI.transposeInPlace();
        ASSERTMSG(checkOrthogonality(xDir,yDir,m_zDir,1e-6),
                  "Not orthogonal: x:"<<xDir.transpose()<< " y: "<< yDir.transpose() << " z: "  << m_zDir.transpose());

        // Project Points onto xDir,yDir Halfspace
        auto size = points.cols();
        m_p.resize(2,size);
        //m_p = m_A_KI * points;  // Project points! (below is faster)
        Vector3 p;
        m_maxZValue = std::numeric_limits<PREC>::lowest();
        m_minZValue = std::numeric_limits<PREC>::max();
        for(decltype(size) i=0; i<size; ++i) {
            p = m_A_KI*points.col(i);
            m_p.col(i) = p.head<2>();
            m_maxZValue = std::max(m_maxZValue,p(2));
            m_minZValue = std::min(m_minZValue,p(2));
        }


    }

    Matrix2Dyn  m_p; ///< Projected points in frame K

    Vector3 m_zDir;
    Matrix33 m_A_KI; ///< Transformation from I frame into the projection frame K
    PREC m_minZValue, m_maxZValue;

};
};
#endif
