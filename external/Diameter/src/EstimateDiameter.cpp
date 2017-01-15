// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <limits>

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

#include "ApproxMVBB/Diameter/EstimateDiameter.hpp"



// Implementation ===============================================

#include "ApproxMVBB/Diameter/Utils/alloc.hpp"
#include "ApproxMVBB/Diameter/Utils/util.hpp"

namespace ApproxMVBB
{

double DiameterEstimator::estimateDiameter(
        Diameter::TypeSegment *theDiam,
        double const**theList,
        const int first,
        const int last,
        const int dim,
        double epsilon)
{
    using namespace Diameter;
    _SetReductionModeInIterative( 0 );
    _SetReductionModeOfDiameter( 0 );
    _SetReductionModeOfDbleNorm( 0 );
    _DoNotTryToReduceQ();
    return this->estimateDiameterInOneList(theDiam,theList,first,last,dim, epsilon);
}

double DiameterEstimator::estimateDiameterInOneList(
        Diameter::TypeSegment *theDiam,
        double const**theList,
        const int first,
        const int last,
        const int dim,
        double _epsilon_  )
{

    using namespace Diameter;

    int index;

    int f=first;
    int l=last;

    int newEstimateIsSmallerThanCurrentEstimate;
    TypeSegment theSeg;

    TypeListOfSegments theDoubleNormals;

    double newEstimate;

    int newlast;


    int verboseWhenReducing     = _GetVerboseWhenReducing();
    int _reduction_mode_in_iterative_ = _GetReductionModeInIterative();
    int tryToReduceQ            = _GetTryToReduceQ();
    int _reduction_mode_of_diameter_ = _GetReductionModeOfDiameter();
    int _reduction_mode_of_dbleNorm_ = _GetReductionModeOfDbleNorm();
    int _Q_scan_                     = _GetQscan();
    int _tight_bounds_               = _GetTightBounds();

    int i, j, k, n;
    int index1, index2;

    int suspicion_of_convex_hull = 0; // not used
    int fdn, ldn, idn;

    double epsilon = _epsilon_;
    double bound, upperBound = 0.0;

    double upperSquareDiameter = 0.0;

    theDoubleNormals.n = 0;
    theDoubleNormals.nalloc = 0;
    theDoubleNormals.seg = NULL;

    theDiam->extremity1 = (double*)NULL;
    theDiam->extremity2 = (double*)NULL;
    theDiam->squareDiameter = std::numeric_limits<double>::lowest();

    if ( first < 0 || last < 0 ) return( -1.0 );
    if ( first > last )
    {
        l = first;
        f = last;
    }
    if ( f == l )
    {
        theDiam->extremity1 = theList[f];
        theDiam->extremity2 = theList[l];
        return( 0.0 );
    }

    index = getRandomInt( f, l );
    do
    {

        /* end conditions
         */
        newEstimateIsSmallerThanCurrentEstimate = 0;

        /* find a double normal
         */
        newEstimate = _MaximalSegmentInOneList( &theSeg, index, theList, &f, &l, dim );

        /* if we get a better estimation
         */
        if ( newEstimate > theDiam->squareDiameter )
        {

            /* update variables
             */
            *theDiam = theSeg;

            /* keep the maximal segment in list
             */
            if ( _AddSegmentToList( &theSeg, &theDoubleNormals ) != 1 )
            {
                if ( theDoubleNormals.nalloc > 0 ) free( theDoubleNormals.seg );
                return( -1.0 );
            }


            /* find the farthest point outside the sphere
             */
            newlast = l;
            index = _FarthestPointFromSphere( &theSeg, theList,
                    f, &newlast, dim,
                    _reduction_mode_in_iterative_ );
            if ( _reduction_mode_in_iterative_ == 1 )
            {
                if ( verboseWhenReducing )
                    //fprintf( stdout, "...processing frth: remove %d points\n", l-newlast );
                    if ( newlast == l )
                    {
                        suspicion_of_convex_hull = 1;
                        _reduction_mode_of_diameter_ = 0;
                        _reduction_mode_of_dbleNorm_ = 0;
                    }
                l = newlast;
            }

            /* stopping condition
            no point outside the sphere
            */
            if ( index < f )
            {
                if ( theDoubleNormals.nalloc > 0 ) free( theDoubleNormals.seg );
                return( theDiam->squareDiameter );
            }

            /* other stopping condition

            the farthest point M outside the sphere
            is not that far away with a ball of diameter AB (and center C)

            we have MA.MB = MC^2 - |AB|^2 / 4
            leading to 4 * MC^2 = 4 * MA.MB + |AB|^2

            an upper bound of the diameter is then 2 MC
            thus (4 * MA.MB + |AB|^2) is a squared upper bound of the diameter
            */

            bound = 4.0 * _ScalarProduct( theList[index], theDiam->extremity1,
                    theList[index], theDiam->extremity2, dim ) +
                    theDiam->squareDiameter;

            /* stopping condition

            we want 2*MC < (1+epsilon) * d_estimate
                                        d_estimate = |AB|

            4 * MC^2                       < (1+epsilon)^2 * (d_estimate)^2
            4 * MA.MB + (d_estimate)^2     < (1+epsilon)^2 * (d_estimate)^2
            1 + 4 * MA.MB / (d_estimate)^2 < (1+epsilon)^2

            */
            if ( 1.0 + 4.0 * _ScalarProduct( theList[index], theDiam->extremity1,
                    theList[index], theDiam->extremity2, dim ) /
                    theDiam->squareDiameter < ( 1.0 + epsilon ) * ( 1.0 + epsilon ) )
            {
                if ( theDoubleNormals.nalloc > 0 ) free( theDoubleNormals.seg );
                return( bound ) ;
            }

        }
        else
        {

            newEstimateIsSmallerThanCurrentEstimate = 1;

            /*  I add the found segment to the list
            there is no evidence that it is a maximal segment for
            the initial set P (but it is for the current one
            ie the initial minus the points which have been removed),
            it may somehow help in case of reduction of the set
            of potential extremities


            The list of double normals is sorted (from 0 to n)
            by increasing square diameter:
            - the best diameter is added (again) at the end
            - we search the right place for this new element

            */

            if ( _AddSegmentToList( theDiam, &theDoubleNormals ) != 1 )
            {
                if ( theDoubleNormals.nalloc > 0 ) free( theDoubleNormals.seg );
                return( -1.0 );
            }

            for ( n = theDoubleNormals.n-2; n >= 0 ; n-- )
            {
                if ( n == 0 )
                {
                    theDoubleNormals.seg[ n ] = theSeg;
                }
                else
                {
                    if ( theSeg.squareDiameter <= theDoubleNormals.seg[ n ].squareDiameter &&
                            theSeg.squareDiameter >  theDoubleNormals.seg[ n-1 ].squareDiameter )
                    {
                        theDoubleNormals.seg[ n ] = theSeg;
                        n = -1;
                    }
                    else
                    {
                        theDoubleNormals.seg[ n ] = theDoubleNormals.seg[ n-1 ];
                    }
                }
            }

        }

    }
    while ( newEstimateIsSmallerThanCurrentEstimate == 0 );


    /* last processing with the found diameter
       - points inside the smallest sphere of the
         diameter may have been already removed
     */
    if ( _reduction_mode_in_iterative_ > 0 && _reduction_mode_of_diameter_ == 1 )
        _reduction_mode_of_diameter_ = 0;

    newlast = l;
    index = _LastPointOutsideSphereWithDiameter( theDiam, theDiam->squareDiameter,
            theList, f, &newlast, dim,
            _reduction_mode_of_diameter_ );
    if ( _reduction_mode_of_diameter_ == 1 ||
            _reduction_mode_of_diameter_ == 2 )
    {
        if ( verboseWhenReducing )
            //fprintf( stdout, "...processing diam: remove %d points\n", l-newlast );
            if ( newlast == l )
            {
                suspicion_of_convex_hull = 1;
                _reduction_mode_of_dbleNorm_ = 0;
            }
        l = newlast;
    }


    /* in some (rare) case, the remaining points outside the largest
       sphere are removed while searching for a better diameter
       thus it is still an avantageous case.
       if any, we have
       #f       -> #index : points outside the sphere
       #index+1 -> #l     : points inside the sphere
    */
    if ( index < f )
    {
        if ( theDoubleNormals.nalloc > 0 ) free( theDoubleNormals.seg );
        return( theDiam->squareDiameter );
    }

    /* do you have enough precision?
     */
    index2 = index;
    upperSquareDiameter = theDiam->squareDiameter * (1.0 + epsilon) * (1.0 + epsilon);


    index1  = _LastPointOutsideSphereWithDiameter( theDiam, upperSquareDiameter,
            theList, f, &index2, dim, 0 );
    /* there is no points outside the sphere of diameter d * (1 + epsilon)
       find the farthest one to get a better upper bound of the diameter
    */
    if ( index1 < f )
    {
        if ( theDoubleNormals.nalloc > 0 ) free( theDoubleNormals.seg );

        upperBound = 4.0 * _ScalarProduct( theList[f], theDiam->extremity1,
                theList[f], theDiam->extremity2, dim ) +
                theDiam->squareDiameter;

        for ( k=f+1; k<=index2; k++ )
        {
            bound = 4.0 * _ScalarProduct( theList[k], theDiam->extremity1,
                    theList[k], theDiam->extremity2, dim ) +
                    theDiam->squareDiameter;
            if ( upperBound < bound ) upperBound = bound;
        }
        return( upperBound );
    }

    /* get an upper bound of the diameter with points in [#index1+1 -> #index2]
       if there are points in this interval
       else the upper bound is simply d

       we have
       #f        -> #index1 : points outside the sphere of diameter d (1+epsilon)
       #index1+1 -> #index2 : points outside the sphere but inside the previous one
       #index2+1 -> #l      : points inside the sphere
    */
    if ( _tight_bounds_ )
    {
        upperBound = theDiam->squareDiameter;
        if ( index1 < index2 )
        {
            for ( k=index1+1; k<=index2; k++ )
            {
                bound = 4.0 * _ScalarProduct( theList[k], theDiam->extremity1,
                        theList[k], theDiam->extremity2, dim ) +
                        theDiam->squareDiameter;
                if ( upperBound < bound ) upperBound = bound;
            }
        }
    }
    else
    {
        upperBound = upperSquareDiameter;
    }

    /* to get some information on
       the points
    */
    if ( 0 )
    {
        for ( n = theDoubleNormals.n-1; n >= 0; n -- )
        {
            /*_CountPointsInSpheres( &theDoubleNormals.seg[ n ], theDiam->squareDiameter,
                    theList, f, l, dim );*/
        }
    }

    /* here we will reduce the set of potential extremities
       for the diameter,
       ie the set of points which are to be compared against all
       the other points

       right now, we have
       #f        -> #index1 : points outside the sphere of diameter d+epsilon
       #index1+1 -> #index2 : points outside the sphere of diameter d
       #index2+1 -> #l      : points inside the sphere

       we have a set of maximal segments
       theDoubleNormals.seg[ #i ] for #i from 0 to theDoubleNormals.n-1
       with
       theDoubleNormals.seg[ theDoubleNormals.n-1 ] == theDiam

    */
    index = index1;
    /* right now, we have
       #f       -> #index : points outside the sphere of diameter d*(1+epsilon)
       #index+1 -> #l     : points inside the sphere of diameter d*(1+epsilon)
    */

    if ( tryToReduceQ && theDoubleNormals.n > 1 )
    {

        for ( k = 0; k < theDoubleNormals.n; k ++ )
            theDoubleNormals.seg[k].reduction_mode = _reduction_mode_of_dbleNorm_;

        switch ( _Q_scan_ )
        {
        default :
        case 0 :
            /* backward
             */
            ldn = 0;
            fdn = theDoubleNormals.n-2;
            idn = -1;
            break;
        case 1 :
            /* forward
             */
            fdn = 0;
            ldn = theDoubleNormals.n-2;
            idn = +1;
            break;
        }

        for ( n = fdn; n != (ldn+idn) && index >= f ; n += idn )
        {

            /* in [ #f #index ] find the points outside the sphere
            theDoubleNormals.seg[ n ]

            as a result
            #f   -> #i     are to be compared with all other points
            #i+1 -> #index are to be compared with a subset
                          if this subset is empty, continue

            */
            i = _LastPointOutsideSphereWithDiameter( &theDoubleNormals.seg[ n ],
                    upperSquareDiameter,
                    theList, f, &index, dim, 0 );
            if ( i >= index ) continue;


            /* remise a jour de l'upper bound,
            a partir des points de [#i+1     -> #index]
            par rapport a la double normale courante

            Ce sont des points qui etaient candidats parce qu'au
            dela de la precision demandee, mais qui sont maintenant en
            deca de celle-ci pour la double normale courante,
            donc on reste en deca de la precision, par contre je ne sais
            s'il faut toujours mettre a jour la borne sup ...

            */
            if ( _tight_bounds_ )
            {
                for ( k = i+1; k <= index; k++ )
                {
                    bound = 4.0 * _ScalarProduct( theList[k], theDoubleNormals.seg[ n ].extremity1,
                            theList[k], theDoubleNormals.seg[ n ].extremity2, dim ) +
                            theDoubleNormals.seg[ n ].squareDiameter;
                    if ( upperBound < bound ) upperBound = bound;
                }
            }



            /* in [ #index+1 #l ] find the points outside the sphere
            theDoubleNormals.seg[ n ]

            as a result
            #index+1 -> #j   are to be compared with the previous subset
            */

            newlast = l;

            if ( _tight_bounds_ )
            {
                j = _LastPointOutsideSphereAndBoundWithDiameter( &theDoubleNormals.seg[ n ],
                        upperSquareDiameter,
                        theList, index+1, &newlast, dim,
                        theDoubleNormals.seg[ n ].reduction_mode,
                        &bound );
                if ( upperBound < bound ) upperBound = bound;
            }
            else
            {
                j = _LastPointOutsideSphereWithDiameter( &theDoubleNormals.seg[ n ],
                        upperSquareDiameter,
                        theList, index+1, &newlast, dim,
                        theDoubleNormals.seg[ n ].reduction_mode );
            }

            if ( theDoubleNormals.seg[ n ].reduction_mode == 1 ||
                    theDoubleNormals.seg[ n ].reduction_mode == 2 )
            {

                if ( verboseWhenReducing )
                    //fprintf( stdout, "...processing dbNR: remove %d points\n", l-newlast );
                    if ( newlast == l )
                    {
                        suspicion_of_convex_hull = 1;
                        for ( k = 0; k < theDoubleNormals.n; k ++ )
                            theDoubleNormals.seg[k].reduction_mode = 0;
                    }
                l = newlast;
            }

            if ( j <= index )
            {
                index = i;
                continue;
            }

            /* right now
            #f       -> #i     : points to be compared with all other points
            #i+1     -> #index : points to be compared with the below set
            #index+1 -> #j     : points to be compared with the above set
            #j+1     -> #l     : remaining points

            */

            theSeg.extremity1 = (double*)NULL;
            theSeg.extremity2 = (double*)NULL;
            theSeg.squareDiameter = 0.0;


            newEstimate = _QuadraticDiameterInTwoLists( &theSeg, NULL, NULL,
                    theList, i+1, index,
                    theList, index+1, j,
                    dim );




            if ( newEstimate > theDiam->squareDiameter )
            {
                /* update variables
                 */
                *theDiam = theSeg;
                if ( upperBound < newEstimate ) upperBound = newEstimate;
                if ( upperSquareDiameter < newEstimate ) upperSquareDiameter = newEstimate;
                /* we find a better estimate
                   it is perhaps not the diameter, according that one
                   diameter extremity can be in [ #f #i ]
                   The question are :
                   1. have we to look for a maximal segment with these two points
                      or not ?
                      -> Seems not necessary ...
                   2. have we to consider this segment with the others ?
                      -> yes
                         but we can not reduce the whole set with it !!!
                */
                theDoubleNormals.seg[ n ] = theSeg;
                theDoubleNormals.seg[ n ].reduction_mode = 0;
                n -= idn;
            }


            /* Q is now reduced
             */
            index = i;

        }
    }

    if ( theDoubleNormals.nalloc > 0 ) free( theDoubleNormals.seg );

    /* exhautive search

       comparison of points from #f to #index
       against all others points
    */

    if ( dim == 2 )
    {
        for ( i=f;   i<=index; i++ )
            for ( j=i+1; j<=l;     j++ )
            {
                newEstimate = _SquareDistance2D( theList[i], theList[j] );
                if ( newEstimate > theDiam->squareDiameter )
                {
                    theDiam->extremity1 = theList[i];
                    theDiam->extremity2 = theList[j];
                    theDiam->squareDiameter = newEstimate;
                    if ( newEstimate > upperBound ) upperBound = newEstimate;
                }
            }
        return( upperBound );
    }


    if ( dim == 3 )
    {
        for ( i=f;   i<=index; i++ )
            for ( j=i+1; j<=l;     j++ )
            {
                newEstimate = _SquareDistance3D( theList[i], theList[j] );
                if ( newEstimate > theDiam->squareDiameter )
                {
                    theDiam->extremity1 = theList[i];
                    theDiam->extremity2 = theList[j];
                    theDiam->squareDiameter = newEstimate;
                    if ( newEstimate > upperBound ) upperBound = newEstimate;
                }
            }
        return( upperBound );
    }


    for ( i=f;   i<=index; i++ )
        for ( j=i+1; j<=l;     j++ )
        {
            newEstimate = _SquareDistance( theList[i], theList[j], dim );
            if ( newEstimate > theDiam->squareDiameter )
            {
                theDiam->extremity1 = theList[i];
                theDiam->extremity2 = theList[j];
                theDiam->squareDiameter = newEstimate;
                if ( newEstimate > upperBound ) upperBound = newEstimate;
            }
        }
    return( upperBound );
}

}
// ==============================================================

