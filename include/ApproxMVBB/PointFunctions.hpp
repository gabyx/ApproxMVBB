// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_PointFunctions_hpp
#define ApproxMVBB_PointFunctions_hpp

#include <string>

#include ApproxMVBB_AssertionDebug_INCLUDE_FILE
#include ApproxMVBB_StaticAssert_INCLUDE_FILE

#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include "ApproxMVBB/TypeDefsPoints.hpp"

#include "ApproxMVBB/Diameter/EstimateDiameter.hpp"
#include "ApproxMVBB/GeometryPredicates/Predicates.hpp"

namespace ApproxMVBB{
namespace PointFunctions {

    ApproxMVBB_DEFINE_MATRIX_TYPES
    DEFINE_POINTS_CONFIG_TYPES

    template<typename Derived>
    void applyRandomRotTrans(MatrixBase<Derived> & points) {

        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3, Eigen::Dynamic)
        Quaternion q;
        q.coeffs().setRandom();
        q.normalize();
        Matrix33 R = q.matrix();
        Vector3 trans;
        trans.setRandom();
        points = R*points;
        points.colwise() += trans;
        std::cout << "Applied Transformation" << std::endl;
    }

    template<typename VecT1, typename VecT2>
    inline bool almostEqual(const VecT1  & a, const VecT2  & b, PREC eps = 1e-8 ) {
        return ((a-b).array().abs() <= eps).all();
    }
    template<typename VecT1, typename VecT2>
    inline bool equal(const VecT1  & a, const VecT2  & b) {
        return (a.array() == b.array()).all();
    }

    /** vec1 = b-a and vec2 = c-a */
    template<typename VecT1, typename VecT2, typename VecT3>
    inline int areaSign(const VecT1  & a, const VecT2  & b, const VecT3  & c) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT1,2)
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT2,2)
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT3,2)

        PREC f_A = GeometryPredicates::orient2d(const_cast<double*>(a.data()),
                                                const_cast<double*>(b.data()),
                                                const_cast<double*>(c.data()));

        return  ( ( f_A < (double)0.0 )?   -1   :   ( (f_A > (double)0.0)? 1 : 0) );

    }
    /** Get angle measures from x-Axis through point a */
    template<typename VecT1, typename VecT2>
    inline PREC getAngle(const VecT1  & a, const VecT2 & b) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT1,2)
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT2,2)
        Vector2 t = b - a ;
        PREC angle = std::atan2(t(1),t(0));
        if(angle<0.0) {
            angle += 2*M_PI;
        }
        return angle;
    }

    template<typename VecT1, typename VecT2>
    Vector2 intersectLines(const VecT1 & p1, PREC ang1,
                           const VecT2 & p2, PREC ang2, PREC eps = 1e-10) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT1,2)
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT2,2)
        using namespace std;
        // Two lines p1 + a*t1 =  p2 + b*t2;
        // [-a, b]^-1 * (p1-p2) = [t1;t2]
        // p = (p1-p2)
        // =>
        // t1 = px by - bx py / (ay bx - ax by);

        Vector2 a;
        a(0) = cos(ang1);
        a(1) = sin(ang1);

        PREC bx = cos(ang2);
        PREC by = sin(ang2);

        PREC nom = (p1(0)-p2(0))*by  -  (p1(1)-p2(1)) *bx;

        PREC det = a(1)*bx   -  a(0)*by;

        if( det == 0.0 ) { // lines are collinear
            if(abs(nom) < eps ) { // if the two lines are almost identical! rot( p1-p2, 90 degrees) almost orthogonal to b
                return p1;
            }
            a(0) = std::numeric_limits<PREC>::infinity();
            a(1) = std::numeric_limits<PREC>::infinity();
            return a;
        }

        return  (nom / det) *a + p1;
    }

    /** Traverse first y-Axis then if equal check x-Axis */
    template<typename Derived>
    inline unsigned int minPointYX(const MatrixBase<Derived> & points) {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,2, Eigen::Dynamic)
        unsigned int index = 0;
        for(unsigned int i=1; i<points.cols(); ++i) {
            if( points(1,i) < points(1,index) ) {
                index = i;
            } else if( points(1,i) == points(1,index)  && points(0,i) <  points(0,index) ) {
                index = i;
            }
        }
        return index;
    }
    template<unsigned int Dimension,typename TVector,typename Derived>
    std::pair<TVector,TVector> estimateDiameter(const MatrixBase<Derived> & points, const PREC epsilon) {

        ApproxMVBB_STATIC_ASSERT(Derived::RowsAtCompileTime == Dimension);

        MatrixBase<Derived> & pp = const_cast< MatrixBase<Derived> &>(points);

        // Construct pointer list
        auto size = pp.cols();
        PREC* * pList = new PREC*[size];
        for(decltype(size) i=0; i<size; ++i) {
            pList[i] = pp.col(i).data();
        }

        Diameter::typeSegment pairP;
        Diameter::estimateDiameter(&pairP,pList,0,(int)(size-1),Dimension,epsilon);

        MatrixMap<TVector> p1(pairP.extremity1);
        MatrixMap<TVector> p2(pairP.extremity2);

        //    std::cout << "p1: " << p1.transpose() << std::endl
        //              << "p2: " << p2.transpose() << std::endl
        //              << " l: " << std::sqrt(pairP.squareDiameter) << std::endl;
        delete[] pList;
        return std::pair<TVector,TVector>(p1,p2);

    }


    class CompareByAngle {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ApproxMVBB_DEFINE_MATRIX_TYPES

        /** Cosntructor, points is not a temporary, it accepts all sorts of matrix expressions,
        * however the construction of MatrixRef<> might create a temporary but this is stored in m_p!
        */
        template<typename Derived>
        CompareByAngle(const MatrixBase<Derived> & points,
                       const Vector2 &base ): m_p(points), m_base(base) {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,2, Eigen::Dynamic)
        }

        /** True if b is positively rotated from a, stricly weak ordering! */
        bool operator()(const unsigned int & idx1, const unsigned int & idx2 ) {
            using namespace PointFunctions;

            if( idx1 >= m_p.size() || idx2 >= m_p.size()) {
                ApproxMVBB_ERRORMSG(":" << idx1 << "," << idx2 << "," << m_p.size()<< std::endl);
            }

            if( idx1 == idx2 || PointFunctions::equal(m_p.col(idx1),m_p.col(idx2))) {
                return false;
            }

            // Compare by Area Sign (by ascending positive (z-Axis Rotation) angle in x-y Plane)
            // always  insert the smaller index first , and the larger second (as the function is not completely symmetric!
            if(idx1<idx2) {
                int sgn = areaSign( m_base, m_p.col(idx1), m_p.col(idx2) );
                if(sgn != 0) {
                    return (sgn > 0);
                }
            } else {
                int sgn = areaSign( m_base, m_p.col(idx2), m_p.col(idx1) );
                if(sgn != 0) {
                    return (sgn < 0);
                }
            }
            // Compare by Length (smaller length first)
            return (m_p.col(idx1)-m_base).norm() < (m_p.col(idx2)-m_base).norm();
        }

    private:
        const Vector2 m_base;
        const MatrixRef<const Matrix2Dyn> m_p;
    };

    };
};
#endif
