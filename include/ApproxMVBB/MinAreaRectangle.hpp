// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_MinAreaRectangle_hpp
#define ApproxMVBB_MinAreaRectangle_hpp

#include <string>
#include <vector>

#include "ApproxMVBB/Config/Config.hpp"

#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include "ApproxMVBB/TypeDefsPoints.hpp"
#include ApproxMVBB_AssertionDebug_INCLUDE_FILE

#include "ApproxMVBB/AngleFunctions.hpp"
#include "ApproxMVBB/PointFunctions.hpp"
#include "ApproxMVBB/ConvexHull2D.hpp"

namespace ApproxMVBB{
/** Computes the Minimum Area Rectangle of the input points
*   SideEffects: points is aftwards sorted and
*/
class APPROXMVBB_EXPORT MinAreaRectangle {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ApproxMVBB_DEFINE_MATRIX_TYPES

    ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES

    /** Cosntructor, points is not a temporary, it accepts all sorts of matrix expressions,
    * however the construction of MatrixRef<> might create a temporary but this is stored in m_p!
    * MatrixRef<>  m_p is handed further to m_conv
    */
    template<typename Derived>
    MinAreaRectangle(const MatrixBase<Derived> & points)
        : m_p(points), m_convh(m_p) {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,2, Eigen::Dynamic)
        ApproxMVBB_ASSERTMSG( m_p.data() == points.derived().data() ," You store a temporary in a Ref<> which works here, but do you really want this?")
    }

    struct Box2d {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        inline void reset() {
            m_p.setZero();
            m_u.setZero();
            m_v.setZero();
            m_area = 0.0;
            m_uL = 1.0;
            m_vL = 1.0;
        }
        Vector2 m_p;   ///< first corner x = m_p0 + m_u*m_uL * u + m_v*m_vL * v  , u,v in [0,1]
        Vector2 m_u;   ///< vector of first side (x-Axis)  (normalized)
        Vector2 m_v;   ///< vector of second side (y-Axis) (normalized)

        PREC m_uL = 0.0;     ///< Length in direction u
        PREC m_vL = 0.0;     ///< Length direction v

        PREC m_area = 0.0;
    };

    inline const Box2d &
    getMinRectangle() {
        return m_minBox;
    }


    void compute();

private:

    using Vector2U = MyMatrix::Vector2<unsigned int>;

    void computeRectangle();

    struct Caliper {
        unsigned int m_idx = 0;   // index in m_hullIdx
        unsigned int m_ptIdx = 0; // index in m_p
        PREC m_currAngle = 0.0;
    };

    inline void adjustRectangle(){

        // The rectangle might be a slight parallelogram due to numerics
        //     |-l---*
        //  *  +++++++=================+
        //  |  +     |               | +
        //  h  +   | <-v  Rect     |   +
        //  |  + |               |     +
        //  -  +=================+++++++
        //     p        u

        PREC uNorm = m_minBox.m_u.norm();
        PREC vNorm = m_minBox.m_v.norm();

        // Normalize u direction (if u close to zero -> this becomes not finite)
        Vector2 uN = m_minBox.m_u.array() / uNorm;
        Vector2 vN = m_minBox.m_v.array() / vNorm;

        bool uF = uN.allFinite();
        bool vF = vN.allFinite();

        if(uF && vF){

            // make orthogonal (x axis is u)
            Vector2 uNT;
            uNT(0) = -uN(1);
            uNT(1) =  uN(0);

            PREC h =  uNT.dot(m_minBox.m_v);
            PREC l =  uN.dot (m_minBox.m_v);

            if(l>=0.0){
                m_minBox.m_uL = uNorm + l;
            }else{
                m_minBox.m_uL = uNorm - l;
                m_minBox.m_p += uN*l; // move start point back
            }

            // if v vector pointed downwards (negative h)
            if( h<0 ){
                // invert uNT (and switch u,v)
                uNT *= -1.0;
                m_minBox.m_u = uNT;
                m_minBox.m_v = uN;
                m_minBox.m_vL = m_minBox.m_uL;
                m_minBox.m_uL = -h;
            }else{
                m_minBox.m_vL = h;
                m_minBox.m_u = uN;
                m_minBox.m_v = uNT;
            }

        }else if(uF && !vF){
            // set u to normalized
            m_minBox.m_u    = uN;
            // adjust v direction
            m_minBox.m_v(0) = -uN(1);
            m_minBox.m_v(1) =  uN(0);

            m_minBox.m_uL = uNorm;
            m_minBox.m_vL = 0.0;

        }else if(!uF && vF){
            // set v to normalized
            m_minBox.m_v    = vN;

            // adjust u direction
            m_minBox.m_u(0) = -vN(1);
            m_minBox.m_u(1) =  vN(0);

            m_minBox.m_uL = 0.0;
            m_minBox.m_vL = vNorm;
        }else{
            // adjust both directions
            m_minBox.m_u(0) = 1.0; m_minBox.m_u(1) = 0.0;
            m_minBox.m_v(0) = 0.0;  m_minBox.m_v(1) = 1.0;
            m_minBox.m_uL = 0.0; m_minBox.m_vL = 0.0;
        }

    }

    inline void updateCalipers(PREC edgeAngle, Caliper (&c)[4]) {

        updateAngles(edgeAngle, c);
        for(unsigned char i=0; i<4; i++) {
            findVertex(c[i]);
        }

    }

    //determine caliper angles according to the box given with c[0].m_currAngle = edgeAngle;
    inline void updateAngles(PREC edgeAngle, Caliper (&c)[4]) {
        c[0].m_currAngle = AngleFunctions::mapTo2Pi(edgeAngle);
        c[1].m_currAngle = AngleFunctions::mapTo2Pi(c[0].m_currAngle + 0.5*M_PI);
        c[2].m_currAngle = AngleFunctions::mapTo2Pi(c[1].m_currAngle + 0.5*M_PI);
        c[3].m_currAngle = AngleFunctions::mapTo2Pi(c[2].m_currAngle + 0.5*M_PI);

        //        std::cout << "caliper 1 angle:" <<  c[0].m_currAngle << std::endl;
        //        std::cout << "caliper 2 angle:" <<  c[1].m_currAngle << std::endl;
        //        std::cout << "caliper 3 angle:" <<  c[2].m_currAngle << std::endl;
        //        std::cout << "caliper 4 angle:" <<  c[3].m_currAngle << std::endl;
    }

    // determine the vertex v for which the edge angle is greater than c.m_currAngle
    // the find algroithm starts at c.m_idx
    void findVertex(Caliper & c);

    void getBox(Caliper (&c)[4], Box2d & box);


    std::vector<unsigned int> m_hullIdx;

    std::vector<PREC> m_angles;
    Box2d m_minBox;
    const MatrixRef<const Matrix2Dyn> m_p;

    ConvexHull2D m_convh;

};

}
#endif
