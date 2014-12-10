// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef Diameter_Utils_alloc_h
#define Diameter_Utils_alloc_h


#include <stdlib.h>
#include <string.h>

#include "ApproxMVBB/Diameter/TypeSegment.hpp"

namespace Diameter{

extern void *_AllocateListOfPoints( const int n, const int dim );





extern void *_AllocateListOfSegments( const int n );



typedef struct {
  int n;
  int nalloc;
  typeSegment *seg;
} typeListOfSegments;



extern int _AddSegmentToList( typeSegment *s, typeListOfSegments *list );

};

#endif // header guard

