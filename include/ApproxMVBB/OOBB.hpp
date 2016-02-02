// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
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

namespace ApproxMVBB{


class APPROXMVBB_EXPORT OOBB{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ApproxMVBB_DEFINE_MATRIX_TYPES
    ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES

    OOBB() {
        this->reset();
    }

    OOBB(const OOBB &) = default;

    OOBB(const Vector3 & l,
         const Vector3 & u,
         const Matrix33 & A_IK);

    /** Initializes OOBB with AABB3d values, rotation is set to identity!*/
    OOBB(AABB3d & aabb){
        m_minPoint = aabb.m_minPoint;
        m_maxPoint = aabb.m_maxPoint;
        m_q_KI.setIdentity();
    }

    /** Initializes OOBB with AABB3d values, rotation is set to identity!*/
    OOBB & operator=(AABB3d & aabb){
        m_minPoint = aabb.m_minPoint;
        m_maxPoint = aabb.m_maxPoint;
        m_q_KI.setIdentity();
        return *this;
    }

    inline void setZAxisLongest(){
        Vector3::Index i;
        maxExtent(i);
        if(i<2){
            switchZAxis(static_cast<unsigned int>(i));
        }
    }

    /** Switch the z-Axis to the axis with index i (default x-Axis)*/
    void switchZAxis(unsigned int i);

    void reset();

    /** Add point expressed in OOBB's K frame to the OOBB */
    template<typename Derived>
    OOBB & unite(const MatrixBase<Derived> & p){
        m_maxPoint(0) = std::max(m_maxPoint(0),p(0));
        m_maxPoint(1) = std::max(m_maxPoint(1),p(1));
        m_maxPoint(2) = std::max(m_maxPoint(2),p(2));
        m_minPoint(0) = std::min( m_minPoint(0),p(0));
        m_minPoint(1) = std::min( m_minPoint(1),p(1));
        m_minPoint(2) = std::min( m_minPoint(2),p(2));
        return *this;
    }

    inline Vector3 center(){ return 0.5*(m_maxPoint + m_minPoint);}

    inline Array3 extent() const{
        return (m_maxPoint - m_minPoint).array();
    }

    inline PREC maxExtent() const{
        return (m_maxPoint - m_minPoint).maxCoeff();
    }

    inline PREC maxExtent(Vector3::Index & i) const{
        return (m_maxPoint - m_minPoint).maxCoeff(&i);
    }

    inline bool isEmpty() const {
        return m_maxPoint(0) <= m_minPoint(0) || m_maxPoint(1) <= m_minPoint(1) || m_maxPoint(2) <= m_minPoint(2);
    }

    /**
    * Checks if a point overlaps the OOBB
    * @param p input point which is regar
    * @param coordinateSystemIsI determines if the the input point p is in the coordinate system I
             or if false it is regarded to be in the coordinate system of the OOBB, that is, system K
    */
    template<typename Derived, bool coordinateSystemIsI = true>
    inline bool overlaps(const MatrixBase<Derived> &p) const {
        if(coordinateSystemIsI){
            // p is in I frame
            Vector3 t = m_q_KI.inverse() * p; // A_IK^T * I_p
            return ((t.array() >= m_minPoint.array()) && (t.array() <= m_maxPoint.array())).all();
        }else{
            // p is in K Frame!!
            return ((p.array() >= m_minPoint.array()) && (p.array() <= m_maxPoint.array())).all();
        }
    }

    /** Adjust box that all axes have at least a minimal extent of maxExtent*p, if maxExtent*p < eps then all axes to default extent */
    void expandToMinExtentRelative(PREC p = 0.1, PREC defaultExtent = 0.1, PREC eps = 1e-10);

    /** Adjust box that all axes have at least a minimal extent  minExtent*/
    void expandToMinExtentAbsolute(PREC minExtent);

    inline void expand(PREC d) {
        ApproxMVBB_ASSERTMSG(d>=0,"d>=0")
        m_minPoint -= Vector3(d,d,d);
        m_maxPoint += Vector3(d,d,d);
    }

    inline void expand(Vector3 d) {
        ApproxMVBB_ASSERTMSG(d(0)>=0 && d(1)>=0 && d(2)>=0,"d>=0")
        m_minPoint -= d;
        m_maxPoint += d;
    }

    inline PREC volume() const {
        Vector3 d = m_maxPoint- m_minPoint;
        return d(0) * d(1) * d(2);
    }

    /** Get direction vectors in I Frame */
    inline Vector3 getDirection(unsigned int i) const{
        ApproxMVBB_ASSERTMSG(i<3,"Index wrong: " << i)
        Vector3 d = Vector3::Zero();
        d(i) = 1.0;
        return m_q_KI * d; // A_IK* d;
    }

    /** Get all corner points in I Frame
    *   @return A list of all corner points sorted according to (x,y,z) index in K frame
    *   @param coordinateSystemIsI determines if the the output points are represented in the coordinate system I
               or if false they are represented in the coordinate system of the OOBB, that is, system K
    *   (move semantics make this fast)
    */
    template<bool coordinateSystemIsI = true>
    inline Vector3List getCornerPoints() const{
        Vector3List points(8);
        Array3 ex = extent();
        points[0] =   m_minPoint /*+ Array3(0,0,0) * extent*/ ;
        points[1] =   m_minPoint + (Array3(1.0, 0.0, 0.0) * ex).matrix();
        points[2] =   m_minPoint + (Array3(0.0, 1.0, 0.0) * ex).matrix();
        points[3] =   m_minPoint + (Array3(1.0, 1.0, 0.0) * ex).matrix();

        points[4] =   m_minPoint + (Array3(0.0, 0.0, 1.0) * ex).matrix();
        points[5] =   m_minPoint + (Array3(1.0, 0.0, 1.0) * ex).matrix();
        points[6] =   m_minPoint + (Array3(0.0, 1.0, 1.0) * ex).matrix();
        points[7] =   m_maxPoint /*+ Array3(1,1,1) * extent */;

        if(coordinateSystemIsI){
            for(auto & p : points){
                p = (m_q_KI * p).eval(); //    I_p = A_IK * K_p
            }
        }

        return points;
    }

    Quaternion m_q_KI;  ///< Rotation of frame I to frame K, corresponds to a transformation A_IK;
    Vector3 m_minPoint; ///< in K Frame
    Vector3 m_maxPoint; ///< in K Frame
};

}

#endif // OOBB_hpp
