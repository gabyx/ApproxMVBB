// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE

namespace ApproxMVBB{
    Eigen::IOFormat MyMatrixIOFormat::Matlab(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
    Eigen::IOFormat MyMatrixIOFormat::CommaSep(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");
    Eigen::IOFormat MyMatrixIOFormat::SpaceSep(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "", "");
};
