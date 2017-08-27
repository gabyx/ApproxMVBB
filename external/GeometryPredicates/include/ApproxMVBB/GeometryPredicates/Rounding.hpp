// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel N�tzi <nuetzig (at) imes (d0t) mavt (d0t) ethz (d�t) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_GeometryPredicates_Rounding_hpp
#define ApproxMVBB_GeometryPredicates_Rounding_hpp

#include <type_traits>

#include "ApproxMVBB/GeometryPredicates/Config.hpp"
#include "ApproxMVBB/GeometryPredicates/xpfpa.h"

#define STR1(x) #x
#define STRINGIFY(x) STR1((x))

/** Define static state variables for the floating point unit */
#define FPU_DECLARE XPFPA_DECLARE()

/** Switch to double precision (no extendet!) */
#define FPU_ROUND_DOUBLE XPFPA_SWITCH_DOUBLE()

/** Restore to previous state */
#define FPU_RESTORE XPFPA_RESTORE()

static_assert(sizeof(STRINGIFY(FPU_DECLARE)) > 1,
              "You are compiling GeometryPredicates without Floating Point Control!! "
              "The predicates are not so robust anymore if you uncomment this line!"
              "Check the cmake output GeoemtryPredicates!");

#endif
