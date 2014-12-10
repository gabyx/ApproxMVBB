// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================
#ifndef Diameter_EstimateDiameter_hpp
#define Diameter_EstimateDiameter_hpp

#include "ApproxMVBB/Diameter/TypeSegment.hpp"


namespace Diameter{

double estimateDiameter( typeSegment *theDiam,
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

};

#endif
