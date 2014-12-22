// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef GeometryPredicates_Predicates_hpp
#define GeometryPredicates_Predicates_hpp

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
};

#endif
