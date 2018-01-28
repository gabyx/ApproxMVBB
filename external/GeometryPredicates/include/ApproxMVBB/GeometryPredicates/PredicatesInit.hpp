// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_GeometryPredicates_PredicatesInit_hpp
#define ApproxMVBB_GeometryPredicates_PredicatesInit_hpp

#include "ApproxMVBB/GeometryPredicates/Config.hpp"

namespace GeometryPredicates
{
    /** Initializer which intializes essential values for the predicates */
    struct PredicatesInit
    {
        REAL splitter; /* = 2^ceiling(p / 2) + 1.  Used to split floats in half. */
        /* A set of coefficients used to calculate maximum roundoff errors. */
        REAL resulterrbound;
        REAL ccwerrboundA, ccwerrboundB, ccwerrboundC;
        REAL o3derrboundA, o3derrboundB, o3derrboundC;
        REAL iccerrboundA, iccerrboundB, iccerrboundC;
        REAL isperrboundA, isperrboundB, isperrboundC;

        PredicatesInit();
    };

    //! Global variable which contains the init values.
    extern PredicatesInit predicatesInit;
}  // namespace GeometryPredicates

#endif
