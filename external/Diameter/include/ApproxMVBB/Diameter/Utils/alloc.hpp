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

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

#include "ApproxMVBB/Diameter/TypeSegment.hpp"

namespace ApproxMVBB{
namespace Diameter{

APPROXMVBB_EXPORT void *_AllocateListOfPoints( const int n, const int dim );

APPROXMVBB_EXPORT void *_AllocateListOfSegments( const int n );


struct TypeListOfSegments {
  int n;
  int nalloc;
  TypeSegment *seg;
} ;

APPROXMVBB_EXPORT int _AddSegmentToList( TypeSegment *s, TypeListOfSegments *list );

}
}

#endif

