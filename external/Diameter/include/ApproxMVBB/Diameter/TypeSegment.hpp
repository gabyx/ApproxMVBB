// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Diameter_TypeSegment_hpp
#define ApproxMVBB_Diameter_TypeSegment_hpp

namespace Diameter{

    typedef struct {
      double *extremity1;
      double *extremity2;
      double squareDiameter;
      int reduction_mode;
    } typeSegment;

};

#endif // Diameter_TypeSegement_hpp


