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

    OOBB() {
        this->reset();
    }

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

    template<typename Derived, bool transform = true>
    inline bool overlaps(const MatrixBase<Derived> &p) const {
        if(transform){
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
    inline Vector3 getDirection(unsigned int i){
        ApproxMVBB_ASSERTMSG(i<3,"Index wrong: " << i)
        Vector3 d; d.setZero(); d(i) = 1.0;
        return m_q_KI * d; // A_IK* d;
    }

    Quaternion m_q_KI;  ///< Rotation of frame I to frame K, corresponds to a transformation A_IK;
    Vector3 m_minPoint; ///< in K Frame
    Vector3 m_maxPoint; ///< in K Frame
};

}

#endif // OOBB_hpp
