// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_TypeDefsPoints_hpp
#define ApproxMVBB_TypeDefsPoints_hpp

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

namespace ApproxMVBB{
namespace TypeDefsPoints {

    ApproxMVBB_DEFINE_MATRIX_TYPES

    using  Vector3List = StdVecAligned<Vector3>;
    using  Vector2List = StdVecAligned<Vector2>;

    using  Matrix3Dyn = MatrixStatDyn<3>;
    using  Matrix2Dyn = MatrixStatDyn<2>;

}
}

#define ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES \
    using Vector3List = ApproxMVBB::TypeDefsPoints::Vector3List; \
    using Vector2List = ApproxMVBB::TypeDefsPoints::Vector2List;\
    using Matrix3Dyn  = ApproxMVBB::TypeDefsPoints::Matrix3Dyn;\
    using Matrix2Dyn  = ApproxMVBB::TypeDefsPoints::Matrix2Dyn;

#endif
