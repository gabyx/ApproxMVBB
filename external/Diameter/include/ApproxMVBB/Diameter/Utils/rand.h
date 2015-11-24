// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Diameter_Utils_rand_h
#define ApproxMVBB_Diameter_Utils_rand_h

/*************************************************************************
 *  -
 *
 * $Id: rand.h,v 1.2 2004/06/10 09:23:33 greg Exp $
 *
 * Copyright INRIA
 *
 * AUTHOR:
 * Gregoire Malandain (greg@sophia.inria.fr)
 *
 * CREATION DATE:
 * Tue May 16 2000
 *
 *
 * ADDITIONS, CHANGES
 *
 *
 */


#include <math.h>
#include <stdlib.h>

namespace ApproxMVBB{
namespace Diameter{

extern long int _GetRandomCalls();
extern long int _GetRandomSeed();

#ifdef _WIN32
extern void   _SetRandomSeed( unsigned int seed );
#else
extern void   _SetRandomSeed( long int seed );
#endif

/* return a double number d | 0 <= d < 1
 */
extern double _GetRandomDoubleNb( );

/* return a int number i | min <= i <= max
 */
extern int    _GetRandomIntNb( int min, int max );


}
}

#endif

