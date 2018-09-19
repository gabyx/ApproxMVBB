// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================
#ifndef ApproxMVBB_ProjectedPointSet_hpp
#define ApproxMVBB_ProjectedPointSet_hpp

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_AssertionDebug_INCLUDE_FILE
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include "ApproxMVBB/Common/TypeDefsPoints.hpp"
#include "ApproxMVBB/MakeCoordinateSystem.hpp"
#include "ApproxMVBB/PointFunctions.hpp"
#include ApproxMVBB_OOBB_INCLUDE_FILE
#include "ApproxMVBB/MinAreaRectangle.hpp"

//#include "TestFunctions.hpp"

namespace ApproxMVBB
{
    class APPROXMVBB_EXPORT ProjectedPointSet
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ApproxMVBB_DEFINE_MATRIX_TYPES;
        ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

	public:
		ProjectedPointSet();
		~ProjectedPointSet();

	public:
        template<typename Derived>
        OOBB computeMVBBApprox(const Vector3& zDir, const MatrixBase<Derived>& points, const PREC epsilon)
        {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, Eigen::Dynamic);

            using namespace PointFunctions;
            m_zDir = zDir;
            project(points);
            // std::cout <<"projected points" <<std::endl;

            // Estimate diameter in 2d projective plane
            std::pair<Vector2, Vector2> pp = estimateDiameter<2>(m_p, epsilon);

            Vector2 dirX = pp.first - pp.second;
            if((pp.second.array() >= pp.first.array()).all())
            {
                dirX *= -1;
            }
            // std::cout <<"estimated 2d diameter: " << dirX.transpose() << " eps: " <<
            // epsilon << std::endl;
            // Built Coordinate Trafo from coordinate system `K`  to coordinate system `M` 
            Matrix22 A2_MK;
            dirX.normalize();

            // If normalized direction INf/NaN, use (1,0)
            if(!dirX.allFinite())
            {
                dirX.setZero();
                dirX(0) = 1;
            }

            Vector2 dirY(-dirX(1), dirX(0));  // Positive rotation of 90 degrees
            A2_MK.col(0) = dirX;
            A2_MK.col(1) = dirY;
            A2_MK.transposeInPlace();

            AABB2d aabb;
            Vector2 p;
            auto size = m_p.cols();
            for(unsigned int i = 0; i < size; ++i)
            {
                p.noalias() = A2_MK * m_p.col(i);  // Transform all points
                aabb.unite(p);
            }

            //         std::cout << "aabb_min: "<<  aabb.m_minPoint.transpose() <<
            //         std::endl;
            //         std::cout << "aabb_max: "<< aabb.m_maxPoint.transpose() <<
            //         std::endl;

            Vector3 M_min;
            M_min.head<2>() = aabb.m_minPoint;
            M_min(2)        = m_minZValue;

            Vector3 M_max;
            M_max.head<2>() = aabb.m_maxPoint;
            M_max(2)        = m_maxZValue;

            // Make coordinate transformation from `M` coordinate system (Minimum Rectancle)
            // to `K` coordinate system (Projection Plane);
            Matrix33 A_KM;
            A_KM.setIdentity();
            A_KM.block<2, 2>(0, 0) = A2_MK.transpose();

            //         std::cout << "M_min: "<< M_min.transpose() << std::endl;
            //         std::cout << "M_max: "<< M_max.transpose() << std::endl;

            return OOBB(M_min, M_max, m_A_KI.transpose() * A_KM);
        }

        /** Computes the true MVBB by constructing the convex hull in 2d
     * and then building the minimum area rectangle around it and afterwards
     * pulling the rectangle out in direction m_dirZ again which build the MVBB in
     * direction m_dirZ
     */
        template<typename Derived>
        OOBB computeMVBB(const Vector3& zDir, const MatrixBase<Derived>& points)
        {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, Eigen::Dynamic);

            using namespace CoordinateSystem;
            m_zDir = zDir;
            project(points);

            // compute minimum area rectangle first
            // std::cout << "Dump points DEBUG:" << std::endl;
            // TestFunctions::dumpPointsMatrixBinary("DumpedPoints.bin",m_p);

            MinAreaRectangle mar(m_p);
            mar.compute();
            auto rect = mar.getMinRectangle();

            // std::cout << "Dump RECT DEBUG:" << std::endl;
            // Vector2List p;
            // p.push_back( rect.m_p);
            // p.push_back( rect.m_p + rect.m_u );
            // p.push_back( rect.m_p + rect.m_u + rect.m_v );
            // p.push_back( rect.m_p + rect.m_v );
            // p.push_back( rect.m_p);

            // TestFunctions::dumpPoints("./MinAreaRectangleTest13" "Out.txt",p);

            // Box coordinates are in K Frame

            Matrix22 A2_KM;

            //        std::cout << "u:" << rect.m_u.norm() << std::endl;
            //        std::cout << "v:" << rect.m_v.norm() << std::endl;

            A2_KM.col(0) = rect.m_u;
            A2_KM.col(1) = rect.m_v;

            Vector2 M_p = A2_KM.transpose() * rect.m_p;

            Vector3 M_min;
            M_min.head<2>() = M_p;
            M_min(2)        = m_minZValue;

            Vector3 M_max(rect.m_uL, rect.m_vL, 0.0);
            M_max.head<2>() += M_p;
            M_max(2) = m_maxZValue;

            // Make coordinate transformation from `M` coordinate system (Minimum Rectancle)
            // to `K` coordinate system (Projection Plane);
            Matrix33 A_IM;
            // Make A_KM
            A_IM.setIdentity();
            A_IM.block<2, 2>(0, 0) = A2_KM;
            // Make A_IM;
            A_IM = m_A_KI.transpose() * A_IM;  // A_IM = A_IK * A_KM

            return OOBB(M_min, M_max, A_IM);
        }

    private:
        template<typename Derived>
        void project(const MatrixBase<Derived>& points)
        {
            using namespace CoordinateSystem;

            if(points.cols() == 0)
            {
                ApproxMVBB_ERRORMSG("Point set empty!");
            }

            // Generate Orthonormal Bases
            Vector3 xDir, yDir;
            // std::cout << "dir: " <<  m_zDir << std::endl;
            makeCoordinateSystem(m_zDir, xDir, yDir);

            // Make coodinate transform from coordinate system `I`  to K!
            m_A_KI.col(0) = xDir;
            m_A_KI.col(1) = yDir;
            m_A_KI.col(2) = m_zDir;
            m_A_KI.transposeInPlace();
            ApproxMVBB_ASSERTMSG(
                checkOrthogonality(xDir, yDir, m_zDir, 1e-6),
                "Not orthogonal: x:" << xDir.transpose() << " y: " << yDir.transpose() << " z: " << m_zDir.transpose());

            // Project Points onto xDir,yDir Halfspace
            auto size = points.cols();
            m_p.resize(2, size);
            // m_p = m_A_KI * points;  // Project points! (below is faster)
            Vector3 p;
            m_maxZValue = std::numeric_limits<PREC>::lowest();
            m_minZValue = std::numeric_limits<PREC>::max();
            for(decltype(size) i = 0; i < size; ++i)
            {
                p           = m_A_KI * points.col(i);
                m_p.col(i)  = p.head<2>();
                m_maxZValue = std::max(m_maxZValue, p(2));
                m_minZValue = std::min(m_minZValue, p(2));
            }
        }

        Matrix2Dyn m_p;  ///< Projected points in coordinate system `K` 

        Vector3 m_zDir;
        Matrix33 m_A_KI;  ///< Transformation from coordinate system `I`  into the projection coordinate system `K` 
        PREC m_minZValue, m_maxZValue;
    };
}  // namespace ApproxMVBB
#endif
