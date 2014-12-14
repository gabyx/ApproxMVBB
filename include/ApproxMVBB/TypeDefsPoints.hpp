// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
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

};
};

#define DEFINE_POINTS_CONFIG_TYPES \
    using Vector3List = TypeDefsPoints::Vector3List; \
    using Vector2List = TypeDefsPoints::Vector2List;\
    using Matrix3Dyn  = TypeDefsPoints::Matrix3Dyn;\
    using Matrix2Dyn  = TypeDefsPoints::Matrix2Dyn;

#endif
