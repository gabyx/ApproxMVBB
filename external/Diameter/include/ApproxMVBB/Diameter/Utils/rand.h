// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef Diameter_Utils_rand_h
#define Diameter_Utils_rand_h

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

namespace Diameter{

extern long int _GetRandomCalls();
extern long int _GetRandomSeed();

#ifdef WIN32
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


};
#endif // header guard

