// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_AngleFunctions_hpp
#define ApproxMVBB_AngleFunctions_hpp

#include <cmath>
#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
namespace ApproxMVBB{
namespace AngleFunctions {

    ApproxMVBB_DEFINE_MATRIX_TYPES

    /** Maps angle to the range [-pi,pi]
    * Sawtooth function as g = f(x+a/2) - a/2 , f = x - floor(x/a)*a , a = 2*pi
    */
    inline PREC mapToPi(PREC x) {
        return   x -  std::floor( x/(2.0*M_PI) + 0.5 ) * M_PI * 2.0;
    }

    /** Maps angle to the range [0,2*pi]
    * Sawtooth function as f = x - floor(x/a)*a , a = 2*pi
    */
    inline PREC mapTo2Pi(PREC x) {
        return   x -  std::floor( x/(2.0*M_PI)) * M_PI * 2.0;
    }

    /** Relative angle measured from angle */
    inline PREC relativeAnglePi(PREC angle, PREC angle2) {
        return mapToPi(angle2-angle);
    }
    /** Relative angle measured from angle */
    inline PREC relativeAngle2Pi(PREC angle, PREC angle2) {
        return mapTo2Pi(angle2-angle);
    }

};
};
#endif // AngleFunctions_hpp


