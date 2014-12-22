// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
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
