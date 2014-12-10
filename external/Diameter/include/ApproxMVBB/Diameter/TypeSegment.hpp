// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
// ========================================================================================

#ifndef Diameter_TypeSegement_hpp
#define Diameter_TypeSegement_hpp

namespace Diameter{

    typedef struct {
      double *extremity1;
      double *extremity2;
      double squareDiameter;
      int reduction_mode;
    } typeSegment;

};

#endif // Diameter_TypeSegement_hpp


