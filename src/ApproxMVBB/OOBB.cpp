// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_OOBB_INCLUDE_FILE
namespace ApproxMVBB{
OOBB::OOBB(const Vector3 & l,
           const Vector3 & u,
           const Matrix33 & A_IK)
    :m_q_KI(A_IK) {
    m_q_KI.normalize();
    m_minPoint = Vector3(std::min(l(0),u(0)),std::min(l(1),u(1)),std::min(l(2),u(2)));
    m_maxPoint = Vector3(std::max(l(0),u(0)),std::max(l(1),u(1)),std::max(l(2),u(2)));
}

/** Switch the z-Axis to the axis with index i (default x-Axis)*/
void OOBB::switchZAxis(unsigned int i) {
    if(i > 1) {
        return;
    }
    if( i == 0) {
        // Make new x-Axis the z-Axis
        // R_NK = Rotate around 90∞ around Y, and 90∞ around X (always in K frame) )
        // A_IN = A_IK * A_KN = R_KI * R_NK
        m_q_KI = m_q_KI * Quaternion(0.5 ,0.5, 0.5, 0.5);
        // Change points  Coordinates I_[x,y,z] -> K_[y,z,x]
        std::swap(m_minPoint(0),m_minPoint(1));
        std::swap(m_minPoint(1),m_minPoint(2));

        std::swap(m_maxPoint(0),m_maxPoint(1));
        std::swap(m_maxPoint(1),m_maxPoint(2));
    } else {
        // Make new y-Axis the z-Axis
        // R_NK = Rotate around 90∞ around -X, and 90∞ around -Y (always in K frame) )
        // A_IN = A_IK * A_KN = R_KI * R_NK
        m_q_KI = m_q_KI * Quaternion(0.5 ,-0.5, -0.5, -0.5);
        // Change points  Coordinates I_[x,y,z] -> K_[z,x,y]
        std::swap(m_minPoint(0),m_minPoint(2));
        std::swap(m_minPoint(1),m_minPoint(2));

        std::swap(m_maxPoint(0),m_maxPoint(2));
        std::swap(m_maxPoint(1),m_maxPoint(2));
    }

}

void OOBB::expandToMinExtentRelative(PREC p, PREC defaultExtent, PREC eps){
    Array3 e = extent();
    Vector3 c = center();
    Array3::Index idx;
    PREC ext = std::abs(e.maxCoeff(&idx)) * p;

   if( ext < eps ){ // extent of max axis almost zero, set all axis to defaultExtent --> cube
        ext = defaultExtent;
        for(int i=0;i<3;++i){
            m_minPoint(i) = c(i) - 0.5*ext;
            m_maxPoint(i) = c(i) + 0.5*ext;
        }
    }else{
        for(int i=0;i<3;++i){
            if(i!=idx && std::abs(e(i)) < ext){
                m_minPoint(i) = c(i) - 0.5*ext;
                m_maxPoint(i) = c(i) + 0.5*ext;
            }
        }
    }
}

void OOBB::expandToMinExtentAbsolute(PREC minExtent){
    Array3 e = extent();
    Vector3 c = center();

    PREC l = 0.5*minExtent;
    for(int i=0;i<3;++i){
        if(std::abs(e(i)) < minExtent){
            m_minPoint(i) = c(i) - l;
            m_maxPoint(i) = c(i) + l;
        }
    }
}


void OOBB::reset() {
    // Violating the constraint min<max for making a completey empty box!
    m_minPoint(0) = std::numeric_limits<PREC>::max();
    m_maxPoint(0) = std::numeric_limits<PREC>::min();
    m_minPoint(1) = std::numeric_limits<PREC>::max();
    m_maxPoint(1) = std::numeric_limits<PREC>::min();
    m_minPoint(2) = std::numeric_limits<PREC>::max();
    m_maxPoint(2) = std::numeric_limits<PREC>::min();
    m_q_KI.setIdentity();
}
}
