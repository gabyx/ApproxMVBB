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

namespace ApproxMVBB{
class APPROXMVBB_EXPORT OOBB{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ApproxMVBB_DEFINE_MATRIX_TYPES

    OOBB() {
        this->reset();
    };

    OOBB(const Vector3 & l,
         const Vector3 & u,
         const Matrix33 & A_IK);

    inline void setZAxisLongest(){
        typename Vector3::Index i;
        maxExtent(i);
        if(i<2){
            switchZAxis(i);
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

    inline Array3 extent() const{
        return (m_maxPoint - m_minPoint).array();
    };

    inline PREC maxExtent() const{
        return (m_maxPoint - m_minPoint).maxCoeff();
    };

    inline PREC maxExtent(typename Vector3::Index & i) const{
        return (m_maxPoint - m_minPoint).maxCoeff(&i);
    };

    inline bool isEmpty() const {
        return m_maxPoint(0) <= m_minPoint(0) || m_maxPoint(1) <= m_minPoint(1) || m_maxPoint(2) <= m_minPoint(2);
    }

    void expandZeroExtent(PREC percentageOfLongestAxis = 0.1, PREC eps = 1e-10, PREC defaultExtent = 0.1);

    inline void expand(PREC d) {
        ApproxMVBB_ASSERTMSG(d>=0,"d>=0")
        m_minPoint -= Vector3(d,d,d);
        m_maxPoint += Vector3(d,d,d);
    };

    inline void expand(Vector3 d) {
        ApproxMVBB_ASSERTMSG(d(0)>=0 && d(1)>=0 && d(2)>=0,"d>=0")
        m_minPoint -= d;
        m_maxPoint += d;
    };

    inline PREC volume() const {
        Vector3 d = m_maxPoint- m_minPoint;
        return d(0) * d(1) * d(2);
    };

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

};

#endif // OOBB_hpp
