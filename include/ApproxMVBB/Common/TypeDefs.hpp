// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_Common_TypeDefs_hpp
#define ApproxMVBB_Common_TypeDefs_hpp

#include <random>

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_Platform_INCLUDE_FILE
#include ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE

namespace ApproxMVBB{

struct GlobalConfigs {
    using PREC = double;
};

#define ApproxMVBB_DEFINE_MATRIX_TYPES \
    using PREC = typename ApproxMVBB::GlobalConfigs::PREC; \
    ApproxMVBB_DEFINE_MATRIX_TYPES_OF( ApproxMVBB::GlobalConfigs::PREC )


};


#endif



