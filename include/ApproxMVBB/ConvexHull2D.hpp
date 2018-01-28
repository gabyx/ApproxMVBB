// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_ConvexHull2D_hpp
#define ApproxMVBB_ConvexHull2D_hpp

#include <vector>
#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include "ApproxMVBB/Common/TypeDefsPoints.hpp"
#include ApproxMVBB_AssertionDebug_INCLUDE_FILE
#include "ApproxMVBB/ContainerFunctions.hpp"
#include "ApproxMVBB/PointFunctions.hpp"

namespace ApproxMVBB
{
    /** Generate convex hull of 2D points
 * Side Effects: Removes equal points from input point list reference, input
 * points are angle sorted afterwards
 * Function getIndices() returns the ascending indices of the sorted point list
 * which span the convex hull.
 */
    class APPROXMVBB_EXPORT ConvexHull2D
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ApproxMVBB_DEFINE_MATRIX_TYPES;
        ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

        /** Cosntructor, points is not a temporary, it accepts all sorts of matrix
     * expressions,
     * however the construction of MatrixRef<> might create a temporary but
     * this is stored in m_p!
     */
        template<typename Derived>
        ConvexHull2D(const MatrixBase<Derived>& points)
            : m_p(points)
        {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 2, Eigen::Dynamic);
            ApproxMVBB_ASSERTMSG(m_p.data() == points.derived().data(),
                                 " You store a temporary in a Ref<> which works here, "
                                 "but do you really want this?")
        }

        void compute();

        void computeMonotonChain();

        bool verifyHull();

        inline std::vector<unsigned int>& getIndices()
        {
            return m_indicesCH;
        }

    private:
        void chainHull();

        std::vector<unsigned int> m_indicesCH;
        const MatrixRef<const Matrix2Dyn> m_p;
    };
}  // namespace ApproxMVBB
#endif
