// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel N�tzi <nuetzig (at) imes (d0t) mavt (d0t) ethz (d�t) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_GeometryPredicates_Predicates_hpp
#define ApproxMVBB_GeometryPredicates_Predicates_hpp

#include "ApproxMVBB/GeometryPredicates/Config.hpp"

namespace GeometryPredicates
{
    REAL orient2d(REAL* pa, REAL* pb, REAL* pc);
    REAL orient3d(REAL* pa, REAL* pb, REAL* pc, REAL* pd);
    REAL incircle(REAL* pa, REAL* pb, REAL* pc, REAL* pd);
    REAL insphere(REAL* pa, REAL* pb, REAL* pc, REAL* pd, REAL* pe);
}  // namespace GeometryPredicates

#endif
