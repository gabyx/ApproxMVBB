// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Diameter_Utils_alloc_h
#define ApproxMVBB_Diameter_Utils_alloc_h


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

