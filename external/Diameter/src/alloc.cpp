// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================


#include <ApproxMVBB/Diameter/Utils/alloc.hpp>

namespace ApproxMVBB{
namespace Diameter{

void *_AllocateListOfPoints( const int n, const int dim )
{
  void *b = NULL;
  double **list, **dd;
  double *d;
  int i;

  if ( n <= 0 || dim <= 0 ) return(NULL);

  b = (void*)malloc( n         * sizeof( double* )
		     + n * dim * sizeof( double ) );
  if ( b == (void *)NULL )  return( (void *)NULL );

  dd  = list = (double **)b;
  dd += n;

  d   = (double*)dd;
  for (i=0; i<n; i++, d+=dim ) list[i] = d;

  return( b );
}

void *_AllocateListOfSegments( const int n )
{
  void *b = NULL;
  TypeSegment *d;
  int i;


  if ( n <= 0  ) return(NULL);

  b = (void*)malloc( n * sizeof( TypeSegment ) );

  if ( b == (void *)NULL )  return( (void *)NULL );

  d = (TypeSegment *)b;
  for ( i=0; i<n; i++ ) {
    d[i].extremity1 = (double*)NULL;
    d[i].extremity2 = (double*)NULL;
    d[i].squareDiameter = 0.0;
    d[i].reduction_mode = 0;
  }

  return( b );
}

#define _NALLOC_ 20

int _AddSegmentToList( TypeSegment *s, TypeListOfSegments *list )
{
  TypeSegment *d;

  if ( list->nalloc <= 0 )
    list->n = list->nalloc = 0;

  if ( list->n >= list->nalloc ) {

    d = (TypeSegment *)_AllocateListOfSegments( list->nalloc + _NALLOC_ );
    if ( d == NULL ) {
      return( 0 );
    }

    if ( list->nalloc > 0 ) {
      memcpy( d, list->seg, list->nalloc*sizeof(TypeSegment) );
      free( list->seg );
    }

    list->nalloc += _NALLOC_;
    list->seg = d;

  }

  list->seg[list->n] = *s;
  list->n++;

  return( 1 );
}

}
}

