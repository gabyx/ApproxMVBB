// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_OOBB_hpp
#define ApproxMVBB_OOBB_hpp

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include ApproxMVBB_AABB_INCLUDE_FILE
#include "ApproxMVBB/AABB.hpp"

namespace ApproxMVBB
{
    class APPROXMVBB_EXPORT OOBB
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ApproxMVBB_DEFINE_MATRIX_TYPES;
        ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

        /** Default constructor */
        OOBB()
        {
            reset();
        }

        /** Copy and copy-move constructors */
        OOBB(const OOBB&) = default;
        OOBB(OOBB&&)      = default;

        /** Assign and assign-move operators */
        OOBB& operator=(const OOBB&) = default;
        OOBB& operator=(OOBB&&) = default;

        OOBB(const Vector3& minPoint, const Vector3& maxPoint, const Matrix33& A_IK);

        /** Construct this OOBB from an axis aligned bounding box `aabb`.
        The rotation is set to identity! */
        OOBB(AABB3d& aabb)
        {
            m_minPoint = aabb.m_minPoint;
            m_maxPoint = aabb.m_maxPoint;
            m_q_KI.setIdentity();
        }

        /** Construct this OOBB from an axis aligned bounding box `aabb`.
        The rotation is set to identity! */
        OOBB& operator=(AABB3d& aabb)
        {
            m_minPoint = aabb.m_minPoint;
            m_maxPoint = aabb.m_maxPoint;
            m_q_KI.setIdentity();
            return *this;
        }

        /** Set the z-Axis such that it has the longest extent. */
        inline void setZAxisLongest()
        {
            Vector3::Index i;
            maxExtent(i);
            if(i < 2)
            {
                switchZAxis(static_cast<unsigned int>(i));
            }
        }

        void switchZAxis(unsigned int i);
        void reset();

        /** Add points to this OOBB. The points `p` are assumed to be
        represented in this OOBB's coordinate system `K` . */
        template<typename Derived>
        OOBB& unite(const MatrixBase<Derived>& p)
        {
            m_maxPoint(0) = std::max(m_maxPoint(0), p(0));
            m_maxPoint(1) = std::max(m_maxPoint(1), p(1));
            m_maxPoint(2) = std::max(m_maxPoint(2), p(2));
            m_minPoint(0) = std::min(m_minPoint(0), p(0));
            m_minPoint(1) = std::min(m_minPoint(1), p(1));
            m_minPoint(2) = std::min(m_minPoint(2), p(2));
            return *this;
        }

        /** Get the center of the OOBB represented in coordinate system `K` . */
        inline Vector3 center()
        {
            return 0.5 * (m_maxPoint + m_minPoint);
        }

        /** Get the extent of the OOBB represented in coordinate system `K` . */
        inline Array3 extent() const
        {
            return (m_maxPoint - m_minPoint).array();
        }

        /** Get the maximal extent of the OOBB */
        inline PREC maxExtent() const
        {
            return (m_maxPoint - m_minPoint).maxCoeff();
        }

        /** Get the `i`-th maximal extent of the OOBB */
        inline PREC maxExtent(Vector3::Index& i) const
        {
            return (m_maxPoint - m_minPoint).maxCoeff(&i);
        }

        /** Check if the OOBB is empty. */
        inline bool isEmpty() const
        {
            return m_maxPoint(0) <= m_minPoint(0) || m_maxPoint(1) <= m_minPoint(1) || m_maxPoint(2) <= m_minPoint(2);
        }

        /** Checks if a point overlaps the OOBB
        @param p                     Input point.
        @param coordinateSystemIsI   Determines if the the input point is represented in
                                     the coordinate system `I`  or if false they are represented in
                                     the coordinate system of the OOBB (coordinate system `K` ). */
        template<typename Derived, bool coordinateSystemIsI = true>
        inline bool overlaps(const MatrixBase<Derived>& p) const
        {
            if(coordinateSystemIsI)
            {
                // p is in coordinate system `I` 
                Vector3 t = m_q_KI.inverse() * p;  // A_IK^T * I_p
                return ((t.array() >= m_minPoint.array()) && (t.array() <= m_maxPoint.array())).all();
            }
            else
            {
                // p is in K Frame!!
                return ((p.array() >= m_minPoint.array()) && (p.array() <= m_maxPoint.array())).all();
            }
        }

        void expandToMinExtentRelative(PREC p = 0.1, PREC defaultExtent = 0.1, PREC eps = 1e-10);
        void expandToMinExtentAbsolute(PREC minExtent);

        /** Expand the box symmetrically by an absolute value \p d. */
        inline void expand(PREC d)
        {
            ApproxMVBB_ASSERTMSG(d >= 0, "d>=0") m_minPoint -= Vector3(d, d, d);
            m_maxPoint += Vector3(d, d, d);
        }

        /** Expand the box symmetrically in each dimension by the absolute values of the vector \p d. */
        inline void expand(Vector3 d)
        {
            ApproxMVBB_ASSERTMSG(d(0) >= 0 && d(1) >= 0 && d(2) >= 0, "d>=0") m_minPoint -= d;
            m_maxPoint += d;
        }

        /** Get the volume of the box. This might be negative if isEmpty() is true. */
        inline PREC volume() const
        {
            Vector3 d = m_maxPoint - m_minPoint;
            return d(0) * d(1) * d(2);
        }

        /** Get direction vectors in the coordinate system `I` . */
        inline Vector3 getDirection(unsigned int i) const
        {
            ApproxMVBB_ASSERTMSG(i < 3, "Index wrong: " << i) Vector3 d = Vector3::Zero();
            d(i)                                                        = 1.0;
            return m_q_KI * d;  // A_IK* d;
        }

        /** Get all corner points of the OOBB.
        @param coordinateSystemIsI   Determines if the the output points are represented in
                                     the coordinate system `I`  or if false they are represented in
                                     the coordinate system of the OOBB (coordinate system `K` ).
        @return A list of all corner points sorted according to (x,y,z) index in coordinate system `K` . */
        template<bool coordinateSystemIsI = true>
        inline Vector3List getCornerPoints() const
        {
            Vector3List points(8);
            Array3 ex = extent();
            points[0] = m_minPoint /*+ Array3(0,0,0) * extent*/;
            points[1] = m_minPoint + (Array3(1.0, 0.0, 0.0) * ex).matrix();
            points[2] = m_minPoint + (Array3(0.0, 1.0, 0.0) * ex).matrix();
            points[3] = m_minPoint + (Array3(1.0, 1.0, 0.0) * ex).matrix();

            points[4] = m_minPoint + (Array3(0.0, 0.0, 1.0) * ex).matrix();
            points[5] = m_minPoint + (Array3(1.0, 0.0, 1.0) * ex).matrix();
            points[6] = m_minPoint + (Array3(0.0, 1.0, 1.0) * ex).matrix();
            points[7] = m_maxPoint /*+ Array3(1,1,1) * extent */;

            if(coordinateSystemIsI)
            {
                for(auto& p : points)
                {
                    p = (m_q_KI * p).eval();  //    I_p = A_IK * K_p
                }
            }

            return points;
        }

        Quaternion m_q_KI;   /**< Rotation of coordinate system `I`  to the coordinate system `K`  (OOBB),
                              corresponds to a transformation A_IK. */
        Vector3 m_minPoint;  //!< Minimal Point. Represented in coordinate system `K`  (OOBB).
        Vector3 m_maxPoint;  //!< Maximal Point. Represented in coordinate system `K`  (OOBB).
    };
}  // namespace ApproxMVBB

#endif  // OOBB_hpp
