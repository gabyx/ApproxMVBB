// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_OOBB_hpp
#define ApproxMVBB_OOBB_hpp

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

    inline void expand(PREC d) {
        ASSERTMSG(d>=0,"d>=0")
        m_minPoint -= Vector3(d,d,d);
        m_maxPoint += Vector3(d,d,d);
    };

    inline void expand(Vector3 d) {
        ASSERTMSG(d(0)>=0 && d(1)>=0 && d(2)>=0,"d>=0")
        m_minPoint -= d;
        m_maxPoint += d;
    };

    inline PREC volume() const {
        Vector3 d = m_maxPoint- m_minPoint;
        return d(0) * d(1) * d(2);
    };

    /** Get direction vectors in I Frame */
    inline Vector3 getDirection(unsigned int i){
        ASSERTMSG(i<3,"Index wrong: " << i)
        Vector3 d; d.setZero(); d(i) = 1.0;
        return m_q_KI * d; // A_IK* d;
    }


    Quaternion m_q_KI;  ///< Rotation of frame I to frame K, corresponds to a transformation A_IK;
    Vector3 m_minPoint; ///< in K Frame
    Vector3 m_maxPoint; ///< in K Frame
};

};

#endif // OOBB_hpp
