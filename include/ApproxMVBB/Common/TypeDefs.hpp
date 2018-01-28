// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Common_TypeDefs_hpp
#define ApproxMVBB_Common_TypeDefs_hpp

#include <random>

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_Platform_INCLUDE_FILE
#include ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE
#include ApproxMVBB_MyContainerTypeDefs_INCLUDE_FILE

namespace ApproxMVBB
{
    struct GlobalConfigs
    {
        using PREC = double;
    };

#define ApproxMVBB_DEFINE_MATRIX_TYPES            \
    using PREC = ApproxMVBB::GlobalConfigs::PREC; \
    ApproxMVBB_DEFINE_MATRIX_TYPES_OF(PREC);      \
    ApproxMVBB_DEFINE_CONTAINER_TYPES

}  // namespace ApproxMVBB

#    include "ApproxMVBB/Common/TypeDefsPoints.hpp"

#endif
