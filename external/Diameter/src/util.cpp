// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <iostream>
#include <limits>

#include <ApproxMVBB/Diameter/Utils/util.hpp>

#ifdef __clang__
#    pragma clang diagnostic push
#    pragma clang diagnostic ignored "-Wold-style-cast"
#    pragma clang diagnostic ignored "-Wsign-conversion"
#    pragma clang diagnostic ignored "-Wunreachable-code"
#endif

namespace ApproxMVBB
{
    namespace Diameter
    {
        /* partially sort a list of points

   given a "diameter", points which are 'outside'
   the sphere with a given threshold are put at the beginning
   of the list,
   the returned value is the index of the last point 'outside'
   ie
   points from #first    to #index are outside
     "     "    #index+1 to #last  are inside

   If there are no points outside, #first-1 is returned.

   Given a segment [AB], its centre C is (A+B)/2.
   The dot product MA.MB is equal to MC^2 - |AB|^2 / 4

   1. Being naive, ie to test if points are outside
      the ball of diameter [AB], yield to condition
      MA.MB > 0

   2. Being a little smarter, ie to  test if points are outside
      the ball of center (A+B)/2 and diameter D (assume D>|AB|)
      yield to condition
      MA.MB > ( D^2 - |AB|^2 ) / 4

*/

        int _LastPointOutsideSphereWithDiameter(TypeSegment* theSeg,
                                                const double squareDiameter,
                                                const double** theList,
                                                const int first,
                                                int* last,
                                                const int dim,
                                                const int _reduction_mode_)
        {
            int i;
            int index = first - 1;
            int l     = *last;

            double mamb, am2;

            double minThreshold;
            double medThreshold;
            double maxThreshold;

            double ab  = sqrt(theSeg->squareDiameter);
            double ab2 = theSeg->squareDiameter;

            double R  = sqrt(squareDiameter);
            double R2 = squareDiameter;

            if(first > *last)
                return (first - 1);

            if(squareDiameter <= theSeg->squareDiameter)
            {
                maxThreshold = medThreshold = 0.0;
                minThreshold                = -0.23205080756887729352 * theSeg->squareDiameter;
            }
            else
            {
                minThreshold = (R - .86602540378443864676 * ab) * (R - .86602540378443864676 * ab) - 0.25 * ab2;

                medThreshold = 0.5 * ab2 + 0.25 * R2 - .43301270189221932338 * R * sqrt(4 * ab2 - R2);

                maxThreshold = 0.25 * (R2 - ab2);
            }

            /* 0 : no reduction
       1 : just inside the smallest sphere
       2 : complete reduction
    */
            switch(_reduction_mode_)
            {
                case 0:

                    /* NO REDUCTION CASE
             */
                    if(dim == 2)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                            }
                        }
                        return (index);
                    }

                    if(dim == 3)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                            }
                        }
                        return (index);
                    }

                    for(i = first; i <= l; i++)
                    {
                        mamb = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                        if(mamb > maxThreshold)
                        {
                            index++;
                            _SwapPoints(theList, index, i);
                        }
                    }
                    return (index);

                default:
                case 1:

                    /* REDUCTION INSIDE THE SMALLEST SPHERE
             */

                    if(dim == 2)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                            }
                            else if(mamb <= minThreshold)
                            {
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                            }
                        }
                        *last = l;
                        return (index);
                    }

                    if(dim == 3)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                            }
                            else if(mamb <= minThreshold)
                            {
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                            }
                        }
                        *last = l;
                        return (index);
                    }

                    for(i = first; i <= l; i++)
                    {
                        mamb = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                        if(mamb > maxThreshold)
                        {
                            index++;
                            _SwapPoints(theList, index, i);
                        }
                        else if(mamb <= minThreshold)
                        {
                            _SwapPoints(theList, i, l);
                            i--;
                            l--;
                        }
                    }
                    *last = l;
                    return (index);

                case 2:

                    /* COMPLETE REDUCTION

               On suppose implicitement
               que l'ensemble des points est
               compris dans l'intersection des spheres centrees
               sur A et B et de rayon |AB|.
               En effet, les conditions (pour un rayon dans
               l'intervalle [min,med]) pour l'elimination ne le
               verifient pas.
             */
                    if(dim == 2)
                    {
                        if(squareDiameter <= theSeg->squareDiameter)
                        {
                            for(i = first; i <= l; i++)
                            {
                                mamb = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                                if(mamb > maxThreshold)
                                {
                                    index++;
                                    _SwapPoints(theList, index, i);
                                }
                                else if(mamb > minThreshold)
                                {
                                    am2 = _SquareDistance2D(theList[i], theSeg->extremity1);
                                    if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - mamb * mamb < 0)
                                    {
                                        _SwapPoints(theList, i, l);
                                        i--;
                                        l--;
                                    }
                                }
                                else
                                {
                                    /* if ( mamb <= minThreshold ) */
                                    _SwapPoints(theList, i, l);
                                    i--;
                                    l--;
                                }
                            }
                        }
                        else
                        {
                            for(i = first; i <= l; i++)
                            {
                                mamb = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                                if(mamb > maxThreshold)
                                {
                                    index++;
                                    _SwapPoints(theList, index, i);
                                }
                                else if(mamb > medThreshold)
                                {
                                    continue;
                                }
                                else if(mamb > minThreshold)
                                {
                                    am2 = _SquareDistance2D(theList[i], theSeg->extremity1);
                                    if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - (R2 - ab2 - mamb) * (R2 - ab2 - mamb) <
                                       0)
                                    {
                                        _SwapPoints(theList, i, l);
                                        i--;
                                        l--;
                                    }
                                }
                                else
                                {
                                    /* if ( mamb <= minThreshold ) */
                                    _SwapPoints(theList, i, l);
                                    i--;
                                    l--;
                                }
                            }
                        }
                        *last = l;
                        return (index);
                    }

                    if(dim == 3)
                    {
                        if(squareDiameter <= theSeg->squareDiameter)
                        {
                            for(i = first; i <= l; i++)
                            {
                                mamb = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                                if(mamb > maxThreshold)
                                {
                                    index++;
                                    _SwapPoints(theList, index, i);
                                }
                                else if(mamb > minThreshold)
                                {
                                    am2 = _SquareDistance3D(theList[i], theSeg->extremity1);
                                    if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - mamb * mamb < 0)
                                    {
                                        _SwapPoints(theList, i, l);
                                        i--;
                                        l--;
                                    }
                                }
                                else
                                {
                                    /* if ( mamb <= minThreshold ) */
                                    _SwapPoints(theList, i, l);
                                    i--;
                                    l--;
                                }
                            }
                        }
                        else
                        {
                            for(i = first; i <= l; i++)
                            {
                                mamb = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                                if(mamb > maxThreshold)
                                {
                                    index++;
                                    _SwapPoints(theList, index, i);
                                }
                                else if(mamb > medThreshold)
                                {
                                    continue;
                                }
                                else if(mamb > minThreshold)
                                {
                                    am2 = _SquareDistance3D(theList[i], theSeg->extremity1);
                                    if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - (R2 - ab2 - mamb) * (R2 - ab2 - mamb) <
                                       0)
                                    {
                                        _SwapPoints(theList, i, l);
                                        i--;
                                        l--;
                                    }
                                }
                                else
                                {
                                    /* if ( mamb <= minThreshold ) */
                                    _SwapPoints(theList, i, l);
                                    i--;
                                    l--;
                                }
                            }
                        }
                        *last = l;
                        return (index);
                    }

                    if(squareDiameter <= theSeg->squareDiameter)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                            }
                            else if(mamb > minThreshold)
                            {
                                am2 = _SquareDistance(theList[i], theSeg->extremity1, dim);
                                if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - mamb * mamb < 0)
                                {
                                    _SwapPoints(theList, i, l);
                                    i--;
                                    l--;
                                }
                            }
                            else
                            {
                                /* if ( mamb <= minThreshold ) */
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                            }
                        }
                    }
                    else
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                            }
                            else if(mamb > medThreshold)
                            {
                                continue;
                            }
                            else if(mamb > minThreshold)
                            {
                                am2 = _SquareDistance(theList[i], theSeg->extremity1, dim);
                                if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - (R2 - ab2 - mamb) * (R2 - ab2 - mamb) < 0)
                                {
                                    _SwapPoints(theList, i, l);
                                    i--;
                                    l--;
                                }
                            }
                            else
                            {
                                /* if ( mamb <= minThreshold ) */
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                            }
                        }
                    }
                    *last = l;
                    return (index);

                    /* END
               COMPLETE REDUCTION
            */
            }
        }

        int _LastPointOutsideSphereAndBoundWithDiameter(TypeSegment* theSeg,
                                                        const double squareDiameter,
                                                        const double** theList,
                                                        const int first,
                                                        int* last,
                                                        const int dim,
                                                        const int _reduction_mode_,
                                                        double* bound)
        {
            int i;
            int index = first - 1;
            int l     = *last;

            double mamb, am2;

            double minThreshold;
            double medThreshold;
            double maxThreshold;

            double ab  = sqrt(theSeg->squareDiameter);
            double ab2 = theSeg->squareDiameter;

            double R  = sqrt(squareDiameter);
            double R2 = squareDiameter;

            /* bound
       c'est la plus grande valeur pour les points
       l'interieur de la sphere
       si squareDiameter > theSeg->squareDiameter
       cette valeur peut etre positive
    */
            double b = *bound = (-theSeg->squareDiameter * 0.25);

            if(squareDiameter <= theSeg->squareDiameter)
            {
                maxThreshold = medThreshold = 0.0;
                minThreshold                = -0.23205080756887729352 * theSeg->squareDiameter;

                return (
                    _LastPointOutsideSphereWithDiameter(theSeg, squareDiameter, theList, first, last, dim, _reduction_mode_));
            }
            else
            {
                minThreshold = (R - .86602540378443864676 * ab) * (R - .86602540378443864676 * ab) - 0.25 * ab2;

                medThreshold = 0.5 * ab2 + 0.25 * R2 - .43301270189221932338 * R * sqrt(4 * ab2 - R2);

                maxThreshold = 0.25 * (R2 - ab2);
            }

            /* 0 : no reduction
       1 : just inside the smallest sphere
       2 : complete reduction
    */
            switch(_reduction_mode_)
            {
                case 0:

                    /* NO REDUCTION CASE
             */
                    if(dim == 2)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                                continue;
                            }
                            if(b < mamb)
                                b = mamb;
                        }
                        *bound = b;
                        return (index);
                    }

                    if(dim == 3)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                                continue;
                            }
                            if(b < mamb)
                                b = mamb;
                        }
                        *bound = b;
                        return (index);
                    }

                    for(i = first; i <= l; i++)
                    {
                        mamb = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                        if(mamb > maxThreshold)
                        {
                            index++;
                            _SwapPoints(theList, index, i);
                            continue;
                        }
                        if(b < mamb)
                            b = mamb;
                    }
                    *bound = b;
                    return (index);

                default:
                case 1:

                    /* REDUCTION INSIDE THE SMALLEST SPHERE
             */

                    if(dim == 2)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                                continue;
                            }
                            if(mamb <= minThreshold)
                            {
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                                continue;
                            }
                            if(b < mamb)
                                b = mamb;
                        }
                        *last  = l;
                        *bound = b;
                        return (index);
                    }

                    if(dim == 3)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                                continue;
                            }
                            if(mamb <= minThreshold)
                            {
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                                continue;
                            }
                            if(b < mamb)
                                b = mamb;
                        }
                        *last  = l;
                        *bound = b;
                        return (index);
                    }

                    for(i = first; i <= l; i++)
                    {
                        mamb = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                        if(mamb > maxThreshold)
                        {
                            index++;
                            _SwapPoints(theList, index, i);
                            continue;
                        }
                        if(mamb <= minThreshold)
                        {
                            _SwapPoints(theList, i, l);
                            i--;
                            l--;
                            continue;
                        }
                        if(b < mamb)
                            b = mamb;
                    }
                    *last  = l;
                    *bound = b;
                    return (index);

                case 2:

                    /* COMPLETE REDUCTION

               On suppose implicitement
               que l'ensemble des points est
               compris dans l'intersection des spheres centrees
               sur A et B et de rayon |AB|.
               En effet, les conditions (pour un rayon dans
               l'intervalle [min,med]) pour l'elimination ne le
               verifient pas.
             */
                    if(dim == 2)
                    {
                        if(squareDiameter <= theSeg->squareDiameter)
                        {
                            for(i = first; i <= l; i++)
                            {
                                mamb = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                                if(mamb > maxThreshold)
                                {
                                    index++;
                                    _SwapPoints(theList, index, i);
                                    continue;
                                }
                                if(mamb > minThreshold)
                                {
                                    am2 = _SquareDistance2D(theList[i], theSeg->extremity1);
                                    if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - mamb * mamb < 0)
                                    {
                                        _SwapPoints(theList, i, l);
                                        i--;
                                        l--;
                                        continue;
                                    }
                                    if(b < mamb)
                                        b = mamb;
                                    continue;
                                }
                                /* if ( mamb <= minThreshold ) */
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                            }
                        }
                        else
                        {
                            for(i = first; i <= l; i++)
                            {
                                mamb = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                                if(mamb > maxThreshold)
                                {
                                    index++;
                                    _SwapPoints(theList, index, i);
                                    continue;
                                }
                                if(mamb > medThreshold)
                                {
                                    if(b < mamb)
                                        b = mamb;
                                    continue;
                                }
                                if(mamb > minThreshold)
                                {
                                    am2 = _SquareDistance2D(theList[i], theSeg->extremity1);
                                    if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - (R2 - ab2 - mamb) * (R2 - ab2 - mamb) <
                                       0)
                                    {
                                        _SwapPoints(theList, i, l);
                                        i--;
                                        l--;
                                        continue;
                                    }
                                    if(b < mamb)
                                        b = mamb;
                                    continue;
                                }
                                /* if ( mamb <= minThreshold ) */
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                            }
                        }
                        *last  = l;
                        *bound = b;
                        return (index);
                    }

                    if(dim == 3)
                    {
                        if(squareDiameter <= theSeg->squareDiameter)
                        {
                            for(i = first; i <= l; i++)
                            {
                                mamb = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                                if(mamb > maxThreshold)
                                {
                                    index++;
                                    _SwapPoints(theList, index, i);
                                    continue;
                                }
                                if(mamb > minThreshold)
                                {
                                    am2 = _SquareDistance3D(theList[i], theSeg->extremity1);
                                    if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - mamb * mamb < 0)
                                    {
                                        _SwapPoints(theList, i, l);
                                        i--;
                                        l--;
                                        continue;
                                    }
                                    if(b < mamb)
                                        b = mamb;
                                    continue;
                                }
                                /* if ( mamb <= minThreshold ) */
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                            }
                        }
                        else
                        {
                            for(i = first; i <= l; i++)
                            {
                                mamb = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                                if(mamb > maxThreshold)
                                {
                                    index++;
                                    _SwapPoints(theList, index, i);
                                    continue;
                                }
                                if(mamb > medThreshold)
                                {
                                    if(b < mamb)
                                        b = mamb;
                                    continue;
                                }
                                if(mamb > minThreshold)
                                {
                                    am2 = _SquareDistance3D(theList[i], theSeg->extremity1);
                                    if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - (R2 - ab2 - mamb) * (R2 - ab2 - mamb) <
                                       0)
                                    {
                                        _SwapPoints(theList, i, l);
                                        i--;
                                        l--;
                                        continue;
                                    }
                                    if(b < mamb)
                                        b = mamb;
                                    continue;
                                }
                                /* if ( mamb <= minThreshold ) */
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                            }
                        }
                        *last  = l;
                        *bound = b;
                        return (index);
                    }

                    if(squareDiameter <= theSeg->squareDiameter)
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                                continue;
                            }
                            if(mamb > minThreshold)
                            {
                                am2 = _SquareDistance(theList[i], theSeg->extremity1, dim);
                                if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - mamb * mamb < 0)
                                {
                                    _SwapPoints(theList, i, l);
                                    i--;
                                    l--;
                                    continue;
                                }
                                if(b < mamb)
                                    b = mamb;
                                continue;
                            }
                            /* if ( mamb <= minThreshold ) */
                            _SwapPoints(theList, i, l);
                            i--;
                            l--;
                        }
                    }
                    else
                    {
                        for(i = first; i <= l; i++)
                        {
                            mamb = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                            if(mamb > maxThreshold)
                            {
                                index++;
                                _SwapPoints(theList, index, i);
                                continue;
                            }
                            if(mamb > medThreshold)
                            {
                                if(b < mamb)
                                    b = mamb;
                                continue;
                            }
                            if(mamb > minThreshold)
                            {
                                am2 = _SquareDistance(theList[i], theSeg->extremity1, dim);
                                if(3.0 * (am2 * ab2 - (am2 - mamb) * (am2 - mamb)) - (R2 - ab2 - mamb) * (R2 - ab2 - mamb) < 0)
                                {
                                    _SwapPoints(theList, i, l);
                                    i--;
                                    l--;
                                    continue;
                                }
                                if(b < mamb)
                                    b = mamb;
                                continue;
                            }
                            /* if ( mamb <= minThreshold ) */
                            _SwapPoints(theList, i, l);
                            i--;
                            l--;
                        }
                    }
                    *last  = l;
                    *bound = b;
                    return (index);

                    /* END
               COMPLETE REDUCTION
            */
            }
        }

        // void _CountPointsInSpheres( TypeSegment *theSeg,
        //			    const double squareDiameter,
        //			    const double **theList,
        //			    const int first,
        //			    const int last,
        //			    const int dim )
        //{
        //  double minThreshold;
        //  double medThreshold;
        //  double maxThreshold;
        //
        //  double ab  = sqrt( theSeg->squareDiameter );
        //  double ab2 = theSeg->squareDiameter;
        //
        //  double R  = sqrt( squareDiameter );
        //  double R2 = squareDiameter;
        //
        //  int n[5] = {0,0,0,0,0};
        //  int i;
        //
        //  double mamb, am2;
        //
        //  minThreshold = (R - .86602540378443864676*ab)
        //    * (R - .86602540378443864676*ab)
        //    - 0.25 * ab2;
        //
        //  medThreshold = 0.5 * ab2 + 0.25 * R2
        //    - .43301270189221932338 * R * sqrt( 4 * ab2 - R2 );
        //
        //  maxThreshold = 0.25 * (R2 - ab2);
        //
        //
        //  for ( i=first; i<=last; i++ ) {
        //
        //    mamb = _ScalarProduct( theList[i], theSeg->extremity1,
        //			   theList[i], theSeg->extremity2, dim );
        //
        //    if ( mamb > maxThreshold ) {
        //      n[0] ++ ;
        //      continue;
        //    }
        //
        //    if ( mamb > medThreshold ) {
        //      n[1] ++ ;
        //      continue;
        //    }
        //
        //    if ( mamb > minThreshold ) {
        //      am2 = _SquareDistance( theList[i], theSeg->extremity1, dim );
        //      if ( 3.0 * ( am2 * ab2 - (am2 - mamb)*(am2 - mamb) )
        //	   - (R2 - ab2 - mamb)*(R2 - ab2 - mamb) < 0 ) {
        //	n[2] ++;
        //	continue;
        //      }
        //      n[3] ++;
        //      continue;
        //    }
        //
        //    n[4] ++;
        //  }
        //
        //  printf(" diametre courant = %g  -  double normale = %g\n",
        //	 R, ab );
        //  printf(" %8d points dont\n", last-first+1 );
        //  printf(" - %6d : candidats extremites       R=%g\n", n[0], sqrt(maxThreshold+0.25*ab2) );
        //  printf(" - %6d : rien a faire               R=%g\n", n[1], sqrt(medThreshold+0.25*ab2)  );
        //  printf(" - %6d : a tester                   \n", n[2]+n[3] );
        //  printf("   + %6d : a eliminer\n", n[2] );
        //  printf("   + %6d : a conserver\n", n[3] );
        //  printf(" - %6d : elimines directement       R=%g\n", n[4], sqrt(minThreshold+0.25*ab2)  );
        //  printf("----------\n" );
        //  printf(" %8d\n", n[0]+n[1]+n[2]+n[3]+n[4] );
        //
        //}

        /* Find the farthest point from a sphere

   returned value :
   - its index if there are some points outside the sphere
   - else #(first index) - 1

   As the sphere diameter is an approximation of the set diameter,
   points verifying MA.MB <= (1.5 -sqrt(3)) AB^2
   can be removed

*/

        int _FarthestPointFromSphere(
            TypeSegment* theSeg, const double** theList, const int first, int* last, const int dim, const int _reduction_mode_)
        {
            int i, l = (*last);
            int index = first - 1;
            double diff, maxdiff = 0.0;
            /* threshold = 1.5 - sqrt(3)
     */
            double threshold = -0.23205080756887729352 * theSeg->squareDiameter;

            if(l < first)
                return (index);

            switch(_reduction_mode_)
            {
                case 0:

                    /* NO REDUCTION CASE
             */

                    if(dim == 2)
                    {
                        for(i = first; i <= l; i++)
                        {
                            diff = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(maxdiff < diff)
                            {
                                index   = i;
                                maxdiff = diff;
                            }
                        }
                        return (index);
                    }

                    if(dim == 3)
                    {
                        for(i = first; i <= l; i++)
                        {
                            diff = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(maxdiff < diff)
                            {
                                index   = i;
                                maxdiff = diff;
                            }
                        }
                        return (index);
                    }

                    for(i = first; i <= l; i++)
                    {
                        diff = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                        if(maxdiff < diff)
                        {
                            index   = i;
                            maxdiff = diff;
                        }
                    }
                    return (index);

                default:
                case 1:

                    /* REDUCTION INSIDE THE SMALLEST SPHERE
             */

                    /* AB = diameter extremities
               MA.MB = (MC+CA).(MC+CB) = MC^2 + CA.CB + MC ( CB+CA )
               = MC^2 - R^2   + 0
            */
                    if(dim == 2)
                    {
                        for(i = first; i <= l; i++)
                        {
                            diff = _ScalarProduct2D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(diff > maxdiff)
                            {
                                index   = i;
                                maxdiff = diff;
                            }
                            else if(diff <= threshold)
                            {
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                                continue;
                            }
                        }
                        *last = l;
                        return (index);
                    }

                    if(dim == 3)
                    {
                        for(i = first; i <= l; i++)
                        {
                            diff = _ScalarProduct3D(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2);
                            if(maxdiff < diff)
                            {
                                index   = i;
                                maxdiff = diff;
                            }
                            else if(diff <= threshold)
                            {
                                _SwapPoints(theList, i, l);
                                i--;
                                l--;
                                continue;
                            }
                        }
                        *last = l;
                        return (index);
                    }

                    for(i = first; i <= l; i++)
                    {
                        diff = _ScalarProduct(theList[i], theSeg->extremity1, theList[i], theSeg->extremity2, dim);
                        if(maxdiff < diff)
                        {
                            index   = i;
                            maxdiff = diff;
                        }
                        else if(diff <= threshold)
                        {
                            _SwapPoints(theList, i, l);
                            i--;
                            l--;
                            continue;
                        }
                    }
                    *last = l;
                    return (index);
                    /* END
               REDUCTION INSIDE THE SMALLEST SPHERE
             */
            }
        }

        double _MaximalSegmentInTwoLists(TypeSegment* theSeg,
                                         const int index1,
                                         const double** theList1,
                                         int* first1,
                                         int* last1,
                                         const double** theList2,
                                         int* first2,
                                         int* last2,
                                         int dim)
        {
            int f1 = *first1;
            int l1 = *last1;

            int i1 = index1;

            int f2 = *first2;
            int l2 = *last2;

            int i2;

            const double* ref;

            double d, dprevious;

            theSeg->extremity1     = (double*)nullptr;
            theSeg->extremity2     = (double*)nullptr;
            theSeg->squareDiameter = std::numeric_limits<double>::lowest();

            if(*first1 < 0 || *last1 < 0)
                return (-1.0);
            if(*first1 > *last1)
                return (0.0);
            if(*first2 < 0 || *last2 < 0)
                return (-1.0);
            if(*first2 > *last2)
                return (0.0);
            if(*first2 > *last2)
            {
                l2 = *first2;
                f2 = *last2;
            }
            if(index1 < f1 || index1 > l1)
                return (-1.0);

            do
            {
                dprevious = theSeg->squareDiameter;

                /* reference point in list #1
         */
                ref = theList1[i1];
                /* point #i1 will be compared against all other points
           in list #2
           reject it at the beginning of the list
           do not consider it in the future
        */
                _SwapPoints(theList1, f1, i1);
                f1++;

                /* find the furthest point from 'ref'
         */
                d = _MaximalDistanceFromPoint(&i2, ref, theList2, f2, l2, dim);

                /* better estimate
         */
                if(d > theSeg->squareDiameter)
                {
                    theSeg->extremity1     = ref;
                    theSeg->extremity2     = theList2[i2];
                    theSeg->squareDiameter = d;

                    if(f1 <= l1)
                    {
                        dprevious = theSeg->squareDiameter;

                        /* reference point in list #1
                 */
                        ref = theList2[i2];
                        /* point #i2 will be compared against all other points
                   in list #1
                   reject it at the beginning of the list
                   do not consider it in the future
                */
                        _SwapPoints(theList2, f2, i2);
                        f2++;

                        /* find the furthest point from 'ref'
                 */
                        d = _MaximalDistanceFromPoint(&i1, ref, theList1, f1, l1, dim);

                        /* better estimate
                 */
                        if(d > theSeg->squareDiameter)
                        {
                            theSeg->extremity1     = theList1[i1];
                            theSeg->extremity2     = ref;
                            theSeg->squareDiameter = d;
                        }
                    }
                }

            } while(theSeg->squareDiameter > dprevious && f1 <= l1 && f2 <= l2);

            *first1 = f1;
            *last1  = l1;
            *first2 = f2;
            *last2  = l2;
            return (theSeg->squareDiameter);
        }

        double _MaximalSegmentInOneList(
            TypeSegment* theSeg, const int index, const double** theList, int* first, int* last, const int dim)
        {
            int f = *first;
            int l = *last;

            int i = index;
            const double* ref;

            double d, dprevious;

            theSeg->extremity1     = (double*)nullptr;
            theSeg->extremity2     = (double*)nullptr;
            theSeg->squareDiameter = std::numeric_limits<double>::lowest();

            if(f < 0 || l < 0)
                return (-1.0);
            if(f > l)
                return (0.0);
            if(index < f || index > l)
                return (-1.0);
            if(f == l)
            {
                theSeg->extremity1 = theList[i];
                theSeg->extremity2 = theList[i];
                return (0.0);
            }

            do
            {
                dprevious = theSeg->squareDiameter;

                ref = theList[i];
                /* point #i will be compared against all other points
           reject it at the beginning of the list
           do not consider it in the future
        */
                _SwapPoints(theList, f, i);
                f++;

                /* find the furthest point from 'ref'
         */
                d = _MaximalDistanceFromPoint(&i, ref, theList, f, l, dim);
                if(d > theSeg->squareDiameter)
                {
                    theSeg->extremity1     = ref;
                    theSeg->extremity2     = theList[i];
                    theSeg->squareDiameter = d;
                }

            } while(theSeg->squareDiameter > dprevious && f <= l);

            *first = f;
            *last  = l;
            return (theSeg->squareDiameter);
        }

        double _MaximalDistanceFromPoint(
            int* index, const double* ref, const double** theList, const int first, const int last, const int dim)
        {
            int f = first;
            int l = last;
            int i;
            double dmax, d;

            *index = -1;
            if(first < 0 || last < 0)
                return (-1.0);
            if(first > last)
                return (0.0);

            if(dim == 2)
            {
                dmax   = _SquareDistance2D(theList[f], ref);
                *index = f;

                for(i = f + 1; i <= l; i++)
                {
                    d = _SquareDistance2D(theList[i], ref);
                    if(d > dmax)
                    {
                        dmax   = d;
                        *index = i;
                    }
                }
                return (dmax);
            }

            if(dim == 3)
            {
                dmax   = _SquareDistance3D(theList[f], ref);
                *index = f;

                for(i = f + 1; i <= l; i++)
                {
                    d = _SquareDistance3D(theList[i], ref);
                    if(d > dmax)
                    {
                        dmax   = d;
                        *index = i;
                    }
                }
                return (dmax);
            }

            dmax   = _SquareDistance(theList[f], ref, dim);
            *index = f;

            for(i = f + 1; i <= l; i++)
            {
                d = _SquareDistance(theList[i], ref, dim);
                if(d > dmax)
                {
                    dmax   = d;
                    *index = i;
                }
            }
            return (dmax);
        }

        /* Swap two points
 */

        void _SwapPoints(const double** theList, const int i, const int j)
        {
            const double* tmp;
            tmp        = theList[i];
            theList[i] = theList[j];
            theList[j] = tmp;
        }

        static int _base_ = 100000000;

        void _InitCounter(TypeCounter* c)
        {
            c->c1 = c->c2 = 0;
        }
        void _AddToCounter(TypeCounter* c, const int i)
        {
            c->c2 += i;
            while(c->c2 >= _base_)
            {
                c->c2 -= _base_;
                c->c1++;
            }
            while(c->c2 < 0)
            {
                c->c2 += _base_;
                c->c1--;
            }
        }
        double _GetCounterAverage(TypeCounter* c, const int i)
        {
            return ((_base_ / (double)i) * c->c1 + c->c2 / (double)i);
        }

#ifdef _STATS_
        TypeCounter scalarProducts;

        void _InitScalarProductCounter()
        {
            _InitCounter(&scalarProducts);
        }
        static void _IncScalarProductCounter()
        {
            _AddToCounter(&scalarProducts, 1);
        }
        double _GetScalarProductAverage(int n)
        {
            return (_GetCounterAverage(&scalarProducts, n));
        }
#endif

        /* square distance
 */
        double _SquareDistance(const double* a, const double* b, const int dim)
        {
            int i;
            double d = 0.0;
            double ba;
            for(i = 0; i < dim; i++)
            {
                ba = b[i] - a[i];
                d += ba * ba;
            }

#ifdef _STATS_
            _IncScalarProductCounter();
#endif

            return (d);
        }

        double _SquareDistance3D(const double* a, const double* b)
        {
#ifdef _STATS_
            _IncScalarProductCounter();
#endif

            return ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]) + (b[2] - a[2]) * (b[2] - a[2]));
        }

        double _SquareDistance2D(const double* a, const double* b)
        {
#ifdef _STATS_
            _IncScalarProductCounter();
#endif

            return ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
        }

        /* dot product
 */
        double _ScalarProduct(const double* a, const double* b, const double* c, const double* d, const int dim)
        {
            int i;
            double scalar = 0.0;
            double ab, cd;
            for(i = 0; i < dim; i++)
            {
                ab = b[i] - a[i];
                cd = d[i] - c[i];
                scalar += ab * cd;
            }

#ifdef _STATS_
            _IncScalarProductCounter();
#endif

            return (scalar);
        }

        double _ScalarProduct3D(const double* a, const double* b, const double* c, const double* d)
        {
#ifdef _STATS_
            _IncScalarProductCounter();
#endif

            return ((b[0] - a[0]) * (d[0] - c[0]) + (b[1] - a[1]) * (d[1] - c[1]) + (b[2] - a[2]) * (d[2] - c[2]));
        }

        double _ScalarProduct2D(const double* a, const double* b, const double* c, const double* d)
        {
#ifdef _STATS_
            _IncScalarProductCounter();
#endif

            return ((b[0] - a[0]) * (d[0] - c[0]) + (b[1] - a[1]) * (d[1] - c[1]));
        }

        int _FindPointInList(const double** theList, const int first, const int last, double x0, double x1)
        {
            int i, j = -1;
            double e = 1e-5;
            for(i = first; i <= last; i++)
            {
                if(theList[i][0] - e < x0 && theList[i][0] + e > x0 && theList[i][1] - e < x1 && theList[i][1] + e > x1)
                {
                    if(j == -1)
                    {
                        j = i;
                    }
                    else
                    {
                        fprintf(stderr, "found again at #%d\n", i);
                    }
                }
            }
            return (j);
        }

        double _QuadraticDiameterInOneList(
            TypeSegment* theDiam, const double** theList, const int first, const int last, const int dim)
        {
            int i, j;
            double d;

            theDiam->extremity1     = (double*)nullptr;
            theDiam->extremity2     = (double*)nullptr;
            theDiam->squareDiameter = std::numeric_limits<double>::lowest();

            d = 0.0;

            if(dim == 2)
            {
                for(i = first; i <= last - 1; i++)
                    for(j = i + 1; j <= last; j++)
                    {
                        d = _SquareDistance2D(theList[i], theList[j]);
                        if(d > theDiam->squareDiameter)
                        {
                            theDiam->squareDiameter = d;
                            theDiam->extremity1     = theList[i];
                            theDiam->extremity2     = theList[j];
                        }
                    }
                return (theDiam->squareDiameter);
            }

            if(dim == 3)
            {
                for(i = first; i <= last - 1; i++)
                    for(j = i + 1; j <= last; j++)
                    {
                        d = _SquareDistance3D(theList[i], theList[j]);
                        if(d > theDiam->squareDiameter)
                        {
                            theDiam->squareDiameter = d;
                            theDiam->extremity1     = theList[i];
                            theDiam->extremity2     = theList[j];
                        }
                    }
                return (theDiam->squareDiameter);
            }

            for(i = first; i <= last - 1; i++)
                for(j = i + 1; j <= last; j++)
                {
                    d = _SquareDistance(theList[i], theList[j], dim);
                    if(d > theDiam->squareDiameter)
                    {
                        theDiam->squareDiameter = d;
                        theDiam->extremity1     = theList[i];
                        theDiam->extremity2     = theList[j];
                    }
                }

            return (theDiam->squareDiameter);
        }

        double _QuadraticDiameterInTwoLists(TypeSegment* theDiam,
                                            int* index1,
                                            int* index2,
                                            const double** theList1,
                                            const int first1,
                                            const int last1,
                                            const double** theList2,
                                            const int first2,
                                            const int last2,
                                            const int dim)
        {
            int i, j;
            double d;

            /*
    theDiam->extremity1 = (double*)nullptr;
    theDiam->extremity2 = (double*)nullptr;
    theDiam->squareDiameter = std::numeric_limits<double>::lowest();
    */

            if(index1 != nullptr && index2 != nullptr)
            {
                *index1 = first1 - 1;
                *index2 = first2 - 1;
            }

            d = 0.0;

            if(dim == 2)
            {
                for(i = first1; i <= last1; i++)
                    for(j = first2; j <= last2; j++)
                    {
                        d = _SquareDistance2D(theList1[i], theList2[j]);
                        if(d > theDiam->squareDiameter)
                        {
                            theDiam->squareDiameter = d;
                            theDiam->extremity1     = theList1[i];
                            theDiam->extremity2     = theList2[j];
                            if(index1 != nullptr && index2 != nullptr)
                            {
                                *index1 = i;
                                *index2 = j;
                            }
                        }
                    }
                return (theDiam->squareDiameter);
            }

            if(dim == 3)
            {
                for(i = first1; i <= last1; i++)
                    for(j = first2; j <= last2; j++)
                    {
                        d = _SquareDistance3D(theList1[i], theList2[j]);
                        if(d > theDiam->squareDiameter)
                        {
                            theDiam->squareDiameter = d;
                            theDiam->extremity1     = theList1[i];
                            theDiam->extremity2     = theList2[j];
                            if(index1 != nullptr && index2 != nullptr)
                            {
                                *index1 = i;
                                *index2 = j;
                            }
                        }
                    }
                return (theDiam->squareDiameter);
            }

            for(i = first1; i <= last1; i++)
                for(j = first2; j <= last2; j++)
                {
                    d = _SquareDistance(theList1[i], theList2[j], dim);
                    if(d > theDiam->squareDiameter)
                    {
                        theDiam->squareDiameter = d;
                        theDiam->extremity1     = theList1[i];
                        theDiam->extremity2     = theList2[j];
                        if(index1 != nullptr && index2 != nullptr)
                        {
                            *index1 = i;
                            *index2 = j;
                        }
                    }
                }

            return (theDiam->squareDiameter);
        }
    }  // namespace Diameter
}  // namespace ApproxMVBB

#ifdef __clang__
#    pragma clang diagnostic pop
#endif
