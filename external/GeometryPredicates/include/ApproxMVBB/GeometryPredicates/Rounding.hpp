// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef GeometryPredicates_Rounding_hpp
#define GeometryPredicates_Rounding_hpp

#include "ApproxMVBB/GeometryPredicates/Config.hpp"
#include "ApproxMVBB/GeometryPredicates/xpfpa.h"

/** Define static state variables for the floating point unit */
#define FPU_DECLARE  \
    static XPFPA_DECLARE()

/** Switch to double precision (no extendet!) */
#define FPU_ROUND_DOUBLE \
    XPFPA_SWITCH_DOUBLE() \

/** Restore to previous state */
#define FPU_RESTORE  \
    XPFPA_RESTORE()


#endif
