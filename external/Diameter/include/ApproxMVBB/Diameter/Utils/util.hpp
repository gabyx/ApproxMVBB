// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Diameter_Utils_util_hpp
#define ApproxMVBB_Diameter_Utils_util_hpp


#include <cmath>

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE


#include "ApproxMVBB/Diameter/TypeSegment.hpp"
#include "ApproxMVBB/Diameter/Utils/alloc.hpp"

namespace ApproxMVBB{
namespace Diameter{

    APPROXMVBB_EXPORT int _LastPointOutsideSphereWithDiameter( TypeSegment *theSeg,
                     const double squareDiameter,
                     const double **theList,
                     const int first,
                     int *last,
                     const int dim,
                     const int _reduction_mode_ );

    APPROXMVBB_EXPORT int _LastPointOutsideSphereAndBoundWithDiameter( TypeSegment *theSeg,
                         const double squareDiameter,
                         const double **theList,
                         const int first,
                         int *last,
                         const int dim,
                         const int _reduction_mode_,
                         double *bound );

    APPROXMVBB_EXPORT int _FarthestPointFromSphere( TypeSegment *theSeg,
                  const double **theList,
                  const int first,
                  int *last,
                  const int dim,
                  const int _reduction_mode_ );




    APPROXMVBB_EXPORT void _CountPointsInSpheres( TypeSegment *theSeg,
                const double squareDiameter,
                const double **theList,
                const int first,
                const int last,
                const int dim );




    APPROXMVBB_EXPORT double _MaximalSegmentInTwoLists( TypeSegment *theSeg,
                  const int index1,
                  const double **theList1,
                  int *first1,
                  int *last1,
                  const double **theList2,
                  int *first2,
                  int *last2,
                  int dim );

    APPROXMVBB_EXPORT double _MaximalSegmentInOneList( TypeSegment *theSeg,
                    const int index,
                    const double **theList,
                    int *first,
                    int *last,
                    const int dim );

    APPROXMVBB_EXPORT double _MaximalDistanceFromPoint( int *index,
                     const double *ref,
                     const double **theList,
                     const int first,
                     const int last,
                     const int dim );







    APPROXMVBB_EXPORT double _QuadraticDiameterInOneList( TypeSegment *theDiam,
                       const double **theList,
                       const int first,
                       const int last,
                       const int dim );

    APPROXMVBB_EXPORT double _QuadraticDiameterInTwoLists( TypeSegment *theDiam,
                     int   *index1,
                     int   *index2,
                     const double **theList1,
                     const int first1,
                     const int last1,
                     const double **theList2,
                     const int first2,
                     const int last2,
                     const int dim );


    APPROXMVBB_EXPORT void _SwapPoints( const double **theList, const int i, const int j );

    struct TypeCounter{
        int c1;
        int c2;
    } ;

    APPROXMVBB_EXPORT void _InitCounter( TypeCounter *c );
    APPROXMVBB_EXPORT void _AddToCounter( TypeCounter *c, const int i );
    APPROXMVBB_EXPORT double _GetCounterAverage( TypeCounter *c, const int i );

    #ifdef _STATS_
    APPROXMVBB_EXPORT void _InitScalarProductCounter();
    APPROXMVBB_EXPORT double _GetScalarProductAverage( int n );
    #endif

    /* square distances
    */
    APPROXMVBB_EXPORT double _SquareDistance( const double *a, const double *b, const int dim );
    APPROXMVBB_EXPORT double _SquareDistance3D( const double *a, const double *b );
    APPROXMVBB_EXPORT double _SquareDistance2D( const double *a, const double *b );

    /* dot products
    ab.cd = ( (b[i]-a[i]) . (d[i]-c[i]) )
    */
    APPROXMVBB_EXPORT double _ScalarProduct( const double *a, const double *b, const double *c, const double *d, const int dim );
    APPROXMVBB_EXPORT double _ScalarProduct3D( const double *a, const double *b, const double *c, const double *d );
    APPROXMVBB_EXPORT double _ScalarProduct2D( const double *a, const double *b, const double *c, const double *d );


    APPROXMVBB_EXPORT int _FindPointInList( const double **theList,
              const int first,
              const int last,
              double x0,
              double x1 );


}
}

#endif


