// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <ApproxMVBB/Diameter/Utils/alloc.hpp>

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
        void* _AllocateListOfPoints(const int n, const int dim)
        {
            void* b = nullptr;
            double **list, **dd;
            double* d;
            int i;

            if(n <= 0 || dim <= 0)
                return (nullptr);

            b = (void*)malloc(n * sizeof(double*) + n * dim * sizeof(double));
            if(b == (void*)nullptr)
                return ((void*)nullptr);

            dd = list = (double**)b;
            dd += n;

            d = (double*)dd;
            for(i = 0; i < n; i++, d += dim)
                list[i] = d;

            return (b);
        }

        void* _AllocateListOfSegments(const int n)
        {
            void* b = nullptr;
            TypeSegment* d;
            int i;

            if(n <= 0)
                return (nullptr);

            b = (void*)malloc(n * sizeof(TypeSegment));

            if(b == (void*)nullptr)
                return ((void*)nullptr);

            d = (TypeSegment*)b;
            for(i = 0; i < n; i++)
            {
                d[i].extremity1     = (double*)nullptr;
                d[i].extremity2     = (double*)nullptr;
                d[i].squareDiameter = 0.0;
                d[i].reduction_mode = 0;
            }

            return (b);
        }

        int _AddSegmentToList(TypeSegment* s, TypeListOfSegments* list)
        {
            TypeSegment* d;

            if(list->nalloc <= 0)
                list->n = list->nalloc = 0;

            if(list->n >= list->nalloc)
            {
                d = (TypeSegment*)_AllocateListOfSegments(list->nalloc + 20);
                if(d == nullptr)
                {
                    return (0);
                }

                if(list->nalloc > 0)
                {
                    memcpy(d, list->seg, list->nalloc * sizeof(TypeSegment));
                    free(list->seg);
                }

                list->nalloc += 20;
                list->seg = d;
            }

            list->seg[list->n] = *s;
            list->n++;

            return (1);
        }
    }  // namespace Diameter
}  // namespace ApproxMVBB

#ifdef __clang__
#    pragma clang diagnostic pop
#endif
