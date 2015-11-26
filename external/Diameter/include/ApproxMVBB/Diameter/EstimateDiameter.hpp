// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================
#ifndef ApproxMVBB_Diameter_EstimateDiameter_hpp
#define ApproxMVBB_Diameter_EstimateDiameter_hpp

#include "ApproxMVBB/Diameter/TypeSegment.hpp"

namespace ApproxMVBB{
namespace Diameter{

APPROXMVBB_EXPORT double estimateDiameter(typeSegment *theDiam,
                         double **theList,
                         const int first,
                         const int last,
                         const int dim,
                         double epsilon  );


double estimateDiameterInOneList( typeSegment *theDiam,
                                 double **theList,
                                 const int first,
                                 const int last,
                                 const int dim,
                                 double _epsilon_  );

}
}
#endif
