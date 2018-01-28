// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

/** Floating point config file for Predicates.hpp */

#cmakedefine HAVE__FPU_SETCW
#cmakedefine HAVE_FPSETPREC
#cmakedefine HAVE__CONTROLFP
#cmakedefine HAVE__CONTROLFP_S
#cmakedefine HAVE_FPU_INLINE_ASM_X86

namespace GeometryPredicates
{
    using REAL = double; /* float or double */
}
