// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_PointFunctions_hpp
#define ApproxMVBB_PointFunctions_hpp

#include <string>
#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_AssertionDebug_INCLUDE_FILE
#include ApproxMVBB_StaticAssert_INCLUDE_FILE
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include "ApproxMVBB/Common/FloatingPointComparision.hpp"
#include "ApproxMVBB/Common/TypeDefsPoints.hpp"
#include "ApproxMVBB/Diameter/EstimateDiameter.hpp"
#include "ApproxMVBB/GeometryPredicates/Predicates.hpp"

#ifdef __clang__
#    pragma clang diagnostic push
#    pragma clang diagnostic ignored "-Wunused-private-field"
#endif

namespace ApproxMVBB
{
    namespace PointFunctions
    {
        ApproxMVBB_DEFINE_MATRIX_TYPES;
        ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

        template<typename Derived, typename Gen>
        void applyRandomRotTrans(MatrixBase<Derived>& points, Gen& g)
        {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, Eigen::Dynamic);
            Quaternion q;
            q.coeffs() = q.coeffs().unaryExpr(g);  // TODO: Check if q=[0,0,0,0] is
                                                   // correctly normalized !! otherwise
                                                   // crash! NaN
            q.normalize();
            Matrix33 R = q.matrix();
            Vector3 trans;
            trans  = trans.unaryExpr(g);
            points = R * points;
            points.colwise() += trans;
        }

        template<typename Derived>
        void applyRandomRotTrans(MatrixBase<Derived>& points)
        {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, Eigen::Dynamic);
            Quaternion q;
            q.coeffs().setRandom();
            q.normalize();
            Matrix33 R = q.matrix();
            Vector3 trans;
            trans.setRandom();
            points = R * points;
            points.colwise() += trans;
        }

        template<typename VecT1, typename VecT2>
        inline bool almostEqualAbs(const VecT1& a, const VecT2& b, PREC eps = 1.0e-8)
        {
            return ((a - b).array().abs() <= eps).all();
        }

        template<typename VecT1, typename VecT2>
        inline bool almostEqualUlp(const VecT1& a, const VecT2& b /*, PREC eps = 1.0e-8*/)
        {
            bool ret = true;
            for(unsigned int i = 0; i < a.size(); i++)
            {
                ret = ret && FloatingPoint<PREC>(a(i)).AlmostEquals(FloatingPoint<PREC>(b(i)));
            }
            return ret;
        }

        template<typename VecT1, typename VecT2>
        inline bool equal(const VecT1& a, const VecT2& b)
        {
            return (a.array() == b.array()).all();
        }

        /** vec1 = b-a and vec2 = c-a */
        template<typename VecT1, typename VecT2, typename VecT3>
        inline int orient2d(const VecT1& a, const VecT2& b, const VecT3& c)
        {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT1, 2);
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT2, 2);
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT3, 2);

            PREC f_A = GeometryPredicates::orient2d(
                const_cast<double*>(a.data()), const_cast<double*>(b.data()), const_cast<double*>(c.data()));

            return ((f_A < 0.0) ? -1 : ((f_A > 0.0) ? 1 : 0));
        }

        /** vec1 = b-a and vec2 = c-a */
        template<typename VecT1, typename VecT2, typename VecT3>
        inline bool leftTurn(const VecT1& a, const VecT2& b, const VecT3& c)
        {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT1, 2);
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT2, 2);
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT3, 2);
            return orient2d(a, b, c) > 0;
        }

        /** Postcondition: Points need to be collinear */
        template<typename VecT1, typename VecT2, typename VecT3>
        inline int collinearAreOrderedAlongLine(const VecT1& a, const VecT2& b, const VecT3& c)
        {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT1, 2);
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT2, 2);
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT3, 2);

            if(a(0) < b(0))
                return !(c(0) < b(0));
            if(b(0) < a(0))
                return !(b(0) < c(0));
            if(a(1) < b(1))
                return !(c(1) < b(1));
            if(b(1) < a(1))
                return !(b(1) < c(1));
            return true;  // a==b
        }

        /** Get angle measures from x-Axis through point a */
        template<typename VecT1, typename VecT2>
        inline PREC getAngle(const VecT1& a, const VecT2& b)
        {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT1, 2);
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT2, 2);
            Vector2 t  = b - a;
            PREC angle = std::atan2(t(1), t(0));
            if(angle < 0.0)
            {
                angle += 2 * M_PI;
            }
            return angle;
        }

        template<typename VecT1, typename VecT2>
        Vector2 intersectLines(const VecT1& p1, PREC ang1, const VecT2& p2, PREC ang2, PREC eps = 1e-10)
        {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT1, 2);
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecT2, 2);
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

            PREC nom = (p1(0) - p2(0)) * by - (p1(1) - p2(1)) * bx;

            PREC det = a(1) * bx - a(0) * by;

            if(det == 0.0)
            {  // lines are collinear
                if(abs(nom) < eps)
                {  // if the two lines are almost identical! rot( p1-p2,
                    // 90 degrees) almost orthogonal to b
                    return p1;
                }
                a(0) = std::numeric_limits<PREC>::infinity();
                a(1) = std::numeric_limits<PREC>::infinity();
                return a;
            }

            return (nom / det) * a + p1;
        }

        /** Traverse first y-Axis then if equal check x-Axis */
        template<typename Derived>
        inline unsigned int minPointYX(const MatrixBase<Derived>& points)
        {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 2, Eigen::Dynamic);
            unsigned int index = 0;
            for(unsigned int i = 1; i < points.cols(); ++i)
            {
                if(points(1, i) < points(1, index))
                {
                    index = i;
                }
                else if(points(1, i) == points(1, index) && points(0, i) < points(0, index))
                {
                    index = i;
                }
            }
            return index;
        }

        /** Traverse first y-Axis then if equal check x-Axis */
        template<typename Derived>
        inline unsigned int minPointXY(const MatrixBase<Derived>& points)
        {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 2, Eigen::Dynamic);
            unsigned int index = 0;
            for(unsigned int i = 1; i < points.cols(); ++i)
            {
                if(points(0, i) < points(0, index))
                {
                    index = i;
                }
                else if(points(0, i) == points(0, index) && points(1, i) < points(1, index))
                {
                    index = i;
                }
            }
            return index;
        }

        template<unsigned int Dimension, typename Derived>
        auto estimateDiameter(const MatrixBase<Derived>& points,
                              const PREC epsilon,
                              std::size_t seed = RandomGenerators::defaultSeed)
            -> std::pair<VectorStat<Dimension>, VectorStat<Dimension>>
        {
            ApproxMVBB_STATIC_ASSERTM(Derived::RowsAtCompileTime == Dimension,
                                      "input points matrix need to be (Dimension x N) ");
            ApproxMVBB_STATIC_ASSERTM((std::is_same<typename Derived::Scalar, PREC>::value),
                                      "estimate diameter can only accept double so far");

            // Construct pointer list
            auto size          = points.cols();
            PREC const** pList = new PREC const*[size];
            for(decltype(size) i = 0; i < size; ++i)
            {
                pList[i] = points.col(i).data();
            }

            Diameter::TypeSegment pairP;
            DiameterEstimator diamEstimator(seed);
            diamEstimator.estimateDiameter(&pairP, pList, 0, static_cast<int>(size - 1), Dimension, epsilon);

            using Vector2d = MyMatrix::VectorStat<double, Dimension>;
            const MatrixMap<const Vector2d> p1(pairP.extremity1);
            const MatrixMap<const Vector2d> p2(pairP.extremity2);

            ApproxMVBB_MSGLOG_L2("p1: " << p1.transpose() << std::endl
                                        << "p2: " << p2.transpose() << std::endl
                                        << " l: " << std::sqrt(pairP.squareDiameter) << std::endl);

            delete[] pList;
            return {p1, p2};
        }

        class CompareByAngle
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            ApproxMVBB_DEFINE_MATRIX_TYPES;

            using PointData = std::pair<unsigned int, bool>;

            /** Cosntructor, points is not a temporary, it accepts all sorts of matrix
     * expressions,
     * however the construction of MatrixRef<> might create a temporary but this is
     * stored in m_p!
     */
            template<typename Derived>
            CompareByAngle(const MatrixBase<Derived>& points,
                           const Vector2& base,
                           unsigned int baseIdx,
                           unsigned int& deletedPoints)
                : m_p(points), m_base(base), m_baseIdx(baseIdx), m_deletedPoints(deletedPoints)
            {
                EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 2, Eigen::Dynamic);
            }

            /** True if b is positively rotated from a, stricly weak ordering! */
            int operator()(const PointData& point1In, const PointData& point2In)
            {
                using namespace PointFunctions;
                PointData& point1 = const_cast<PointData&>(point1In);
                PointData& point2 = const_cast<PointData&>(point2In);
                unsigned int idx1 = point1.first;
                unsigned int idx2 = point2.first;
                //
                //            if(idx1<idx2){
                //                if(almostEqualUlp(m_p.col(idx1),m_p.col(idx2))){
                //                    if(!point1.second){point1.second = true;
                //                    ++m_deletedPoints;}
                //                    return false;
                //                }
                //            }else{
                //                if(almostEqualUlp(m_p.col(idx2),m_p.col(idx1))){
                //                    if(!point2.second){point2.second = true;
                //                    ++m_deletedPoints;}
                //                    return false;
                //                }
                //            }

                // Compare by Area Sign (by ascending positive (z-Axis Rotation) angle in
                // x-y Plane)
                // always  insert the smaller index first , and the larger second (as the
                // function is not completely symmetric!
                if(idx1 < idx2)
                {
                    int sgn = orient2d(m_base, m_p.col(idx1), m_p.col(idx2));
                    if(sgn != 0)
                    {
                        return (sgn > 0);
                    }
                }
                else
                {
                    int sgn = orient2d(m_base, m_p.col(idx2), m_p.col(idx1));
                    if(sgn != 0)
                    {
                        return (sgn < 0);
                    }
                }
                // points are collinear

                if(PointFunctions::equal(m_base, m_p.col(idx1)))
                    return false;
                if(PointFunctions::equal(m_base, m_p.col(idx2)))
                    return true;
                if(PointFunctions::equal(m_p.col(idx1), m_p.col(idx2)))
                    return false;

                // if idx2 lies between mbase and idx1 then it should go after idx1
                return collinearAreOrderedAlongLine(m_base, m_p.col(idx2), m_p.col(idx1));
            }

        private:
            const MatrixRef<const Matrix2Dyn> m_p;
            const Vector2 m_base;
            const unsigned int m_baseIdx;
            unsigned int& m_deletedPoints;
        };
    }  // namespace PointFunctions
}  // namespace ApproxMVBB

#    ifdef __clang__
#        pragma clang diagnostic pop
#    endif

#endif
