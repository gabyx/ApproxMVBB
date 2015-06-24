// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Diameter_Utils_util_h
#define ApproxMVBB_Diameter_Utils_util_h

/*************************************************************************
 *  -
 *
 * $Id: util.h,v 1.2 2004/06/10 09:23:33 greg Exp $
 *
 * Copyright INRIA
 *
 * AUTHOR:
 * Gregoire Malandain (greg@sophia.inria.fr)
 *
 * CREATION DATE:
 * Mon May 15 2000
 *
 *
 * ADDITIONS, CHANGES
 *
 *
 */


#include <math.h>

#include "ApproxMVBB/Diameter/TypeSegment.hpp"
#include "ApproxMVBB/Diameter/Utils/alloc.h"

namespace ApproxMVBB{
namespace Diameter{

extern void _VerboseWhenReducing();
extern void _NoVerboseWhenReducing();
extern  int _GetVerboseWhenReducing();

extern void _SetTryToReducePInIterative( int t );
extern void _DoTryToReducePInIterative();
extern void _DoNotTryToReducePInIterative();

extern void _SetReductionModeInIterative( int m );
extern  int _GetReductionModeInIterative();
extern void _SetReductionModeOfDiameter( int m );
extern  int _GetReductionModeOfDiameter();
extern void _SetReductionModeOfDbleNorm( int m );
extern  int _GetReductionModeOfDbleNorm();


extern void _DoTryToReduceQ();
extern void _DoNotTryToReduceQ();
extern  int _GetTryToReduceQ();


extern void _SetQscanToForward();
extern void _SetQscanToBackward();
extern  int _GetQscan();

extern void _DoTryToGetTightBounds();
extern void _DoNotTryToGetTightBounds();
extern  int _GetTightBounds();

extern int _LastPointOutsideSphereWithDiameter( typeSegment *theSeg,
					 const double squareDiameter,
					 double **theList,
					 const int first,
					 int *last,
					 const int dim,
					 const int _reduction_mode_ );

extern int _LastPointOutsideSphereAndBoundWithDiameter( typeSegment *theSeg,
						 const double squareDiameter,
						 double **theList,
						 const int first,
						 int *last,
						 const int dim,
						 const int _reduction_mode_,
						 double *bound );

extern int _FarthestPointFromSphere( typeSegment *theSeg,
			      double **theList,
			      const int first,
			      int *last,
			      const int dim,
			      const int _reduction_mode_ );




extern void _CountPointsInSpheres( typeSegment *theSeg,
			    const double squareDiameter,
			    double **theList,
			    const int first,
			    const int last,
			    const int dim );




extern double _MaximalSegmentInTwoLists( typeSegment *theSeg,
				  const int index1,
				  double **theList1,
				  int *first1,
				  int *last1,
				  double **theList2,
				  int *first2,
				  int *last2,
				  int dim );

extern double _MaximalSegmentInOneList( typeSegment *theSeg,
					const int index,
					double **theList,
					int *first,
					int *last,
					const int dim );

extern double _MaximalDistanceFromPoint( int *index,
					 const double *ref,
					 double **theList,
					 const int first,
					 const int last,
					 const int dim );







extern double _QuadraticDiameterInOneList( typeSegment *theDiam,
					   double **theList,
					   const int first,
					   const int last,
					   const int dim );

extern double _QuadraticDiameterInTwoLists( typeSegment *theDiam,
				     int   *index1,
				     int   *index2,
				     double **theList1,
				     const int first1,
				     const int last1,
				     double **theList2,
				     const int first2,
				     const int last2,
				     const int dim );








extern void _SwapPoints( double **theList, const int i, const int j );





typedef struct {
  int c1;
  int c2;
} typeCounter;

extern void _InitCounter( typeCounter *c );
extern void _AddToCounter( typeCounter *c, const int i );
extern double _GetCounterAverage( typeCounter *c, const int i );

#ifdef _STATS_
extern void _InitScalarProductCounter();
extern double _GetScalarProductAverage( int n );
#endif




/* square distances
 */
extern double _SquareDistance( const double *a, const double *b, const int dim );
extern double _SquareDistance3D( const double *a, const double *b );
extern double _SquareDistance2D( const double *a, const double *b );

/* dot products
   ab.cd = ( (b[i]-a[i]) . (d[i]-c[i]) )
*/
extern double _ScalarProduct( const double *a, const double *b,
		       const double *c, const double *d, const int dim );
extern double _ScalarProduct3D( const double *a, const double *b,
			 const double *c, const double *d );
extern double _ScalarProduct2D( const double *a, const double *b,
			 const double *c, const double *d );


extern int _FindPointInList( double **theList,
		      const int first,
		      const int last,
		      double x0,
		      double x1 );


}
}

#endif


