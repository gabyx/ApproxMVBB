// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_GeometryPredicates_Predicates_hpp
#define ApproxMVBB_GeometryPredicates_Predicates_hpp

namespace GeometryPredicates{

    /* On some machines, the exact arithmetic routines might be defeated by the  */
    /*   use of internal extended precision floating-point registers.  Sometimes */
    /*   this problem can be fixed by defining certain values to be volatile,    */
    /*   thus forcing them to be stored to memory and rounded off.  This isn't   */
    /*   a great solution, though, as it slows the arithmetic down.              */
    /*                                                                           */
    /* To try this out, write "#define INEXACT volatile" below.  Normally,       */
    /*   however, INEXACT should be defined to be nothing.  ("#define INEXACT".) */

    #define INEXACT                          /* Nothing */
    /* #define INEXACT volatile */

    #define REAL double                      /* float or double */
    #define REALPRINT doubleprint
    #define REALRAND doublerand
    #define NARROWRAND narrowdoublerand
    #define UNIFORMRAND uniformdoublerand


    REAL orient2d(REAL * pa,
                    REAL * pb,
                    REAL * pc);
    REAL orient3d(REAL * pa,
                    REAL * pb,
                    REAL * pc,
                    REAL * pd);
    REAL incircle(REAL * pa,
                    REAL * pb,
                    REAL * pc,
                    REAL * pd);
    REAL insphere(REAL * pa,
                    REAL * pb,
                    REAL * pc,
                    REAL * pd,
                    REAL * pe);
}

#endif
