// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_AngleFunctions_hpp
#define ApproxMVBB_AngleFunctions_hpp

#define _USE_MATH_DEFINES
#include <cmath>
#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

#define _USE_MATH_DEFINES
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

namespace ApproxMVBB
{
    namespace AngleFunctions
    {
        ApproxMVBB_DEFINE_MATRIX_TYPES;

        /** Maps angle to the range [-pi,pi]
     * Sawtooth function as g = f(x+a/2) - a/2 , f = x - floor(x/a)*a , a = 2*pi
     */
        inline PREC
        mapToPi(PREC x)
        {
            return x - std::floor(x / (2.0 * M_PI) + 0.5) * M_PI * 2.0;
        }

        /** Maps angle to the range [0,2*pi]
 * Sawtooth function as f = x - floor(x/a)*a , a = 2*pi
 */
        inline PREC mapTo2Pi(PREC x)
        {
            return x - std::floor(x / (2.0 * M_PI)) * M_PI * 2.0;
        }

        /** Relative angle measured from angle */
        inline PREC relativeAnglePi(PREC angle, PREC angle2)
        {
            return mapToPi(angle2 - angle);
        }
        /** Relative angle measured from angle */
        inline PREC relativeAngle2Pi(PREC angle, PREC angle2)
        {
            return mapTo2Pi(angle2 - angle);
        }
    }  // namespace AngleFunctions
}  // namespace ApproxMVBB
#endif  // AngleFunctions_hpp
